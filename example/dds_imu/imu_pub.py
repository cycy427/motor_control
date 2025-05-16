# by mrtang
# 2025/4/12

import sys
import serial
from parsers.hipnuc_serial_parser import hipnuc_parser
from parsers.hipnuc_nmea_parser import hipnuc_nmea_parser

import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.sensor as sensormsg
from copy import deepcopy

import threading

IMUPUBDATATOPIC = "/nubot/z1/imupubdata"

class Z1IMUClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1IMUClient, self).__init__()

        # 设置串口号和波特率
        self._port = '/dev/ttyUSB0'  # 修改为你实际使用的串口号
        self._baudrate = 921600     # 修改为你需要的波特率
        self.latest_hipnuc_frame = None
        self.latest_nmea_frames = []
        self.frame_rate = 0

        self.statetopic = statetopic

        self.daemon = True

        self.imuStates = sensormsg.imudata(orientation=sensormsg.orientation(x=0, y=0, z=0, w=0),
                                            angular_velocity=sensormsg.angular_velocity(x=0, y=0, z=0),
                                            linear_acceleration=sensormsg.linear_acceleration(x=0, y=0, z=0),
                                            Magnetic=sensormsg.Magnetic(x=0, y=0, z=0),
                                            euler_angles=sensormsg.euler_angles(roll=0, pitch=0, yaw=0))

        self._imuStates = sensormsg.imudata(orientation=sensormsg.orientation(x=0, y=0, z=0, w=0),
                                            angular_velocity=sensormsg.angular_velocity(x=0, y=0, z=0),
                                            linear_acceleration=sensormsg.linear_acceleration(x=0, y=0, z=0),
                                            Magnetic=sensormsg.Magnetic(x=0, y=0, z=0),
                                            euler_angles=sensormsg.euler_angles(roll=0, pitch=0, yaw=0))

        self.running = True
        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()
        self.start()

    def run(self):
        # 创建域参与者
        participant = DomainParticipant(0)
        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.statetopic, sensormsg.imudata)
        cmdqos = Qos(
            Policy.Reliability.BestEffort,  # 或 Policy.Reliability.Reliable
            Policy.Durability.Volatile,  # 或 Policy.Durability.TransientLocal
            Policy.History.KeepLast(5),  # 保留最后5条消息
        )
        publisher = Publisher(participant)
        writer = DataWriter(publisher, cmdtopic, qos=cmdqos)

        serial_parser = hipnuc_parser()
        nmea_parser = hipnuc_nmea_parser()

        frame_count = 0
        last_frame_time = time.time()

        try:
            with serial.Serial(self._port, self._baudrate, timeout=1) as ser:
                print(f"Connected to {self._port} at {self._baudrate} baud.")

                while self.running:
                    ## 读取数据
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)

                        try:
                            hipnuc_frames = serial_parser.parse(data)
                            nmea_frames = nmea_parser.parse(data.decode('ascii', errors='ignore'))

                            frame_count += len(hipnuc_frames) + len(nmea_frames)

                            if hipnuc_frames:
                                self.latest_hipnuc_frame = hipnuc_frames[-1]
                            if nmea_frames:
                                self.latest_nmea_frames = nmea_frames

                            current_time = time.time()
                            if current_time - last_frame_time >= 1.0:
                                self.frame_rate = frame_count
                                frame_count = 0
                                last_frame_time = current_time

                            ## 处理数据
                            if self.latest_hipnuc_frame:
                                frame = self.latest_hipnuc_frame

                                # 提取 quaternion 或从 euler 构造
                                quat = frame.quat if frame.quat else [0, 0, 0, 1]
                                roll = frame.roll if frame.roll is not None else 0.0
                                pitch = frame.pitch if frame.pitch is not None else 0.0
                                yaw = frame.yaw if frame.yaw is not None else 0.0

                                # 更新 imuStates 数据
                                self._imuStates.orientation.x = quat[1]
                                self._imuStates.orientation.y = quat[2]
                                self._imuStates.orientation.z = quat[3]
                                self._imuStates.orientation.w = quat[0]

                                self._imuStates.angular_velocity.x = frame.gyr[0] if frame.gyr else 0.0
                                self._imuStates.angular_velocity.y = frame.gyr[1] if frame.gyr else 0.0
                                self._imuStates.angular_velocity.z = frame.gyr[2] if frame.gyr else 0.0

                                self._imuStates.linear_acceleration.x = frame.acc[0] if frame.acc else 0.0
                                self._imuStates.linear_acceleration.y = frame.acc[1] if frame.acc else 0.0
                                self._imuStates.linear_acceleration.z = frame.acc[2] if frame.acc else 1.0

                                self._imuStates.Magnetic.x = frame.mag[0] if frame.mag else 0.0
                                self._imuStates.Magnetic.y = frame.mag[1] if frame.mag else 0.0
                                self._imuStates.Magnetic.z = frame.mag[2] if frame.mag else 0.0

                                self._imuStates.euler_angles.roll = roll
                                self._imuStates.euler_angles.pitch = pitch
                                self._imuStates.euler_angles.yaw = yaw

                            ## 处理发布
                            self._lockcmd.acquire()
                            writer.write(self._imuStates)
                            self._lockcmd.release()
                            time.sleep(0.001)  # 1000Hz

                        except Exception as e:
                            print(f"Error parsing data: {e}")

                    time.sleep(0.001)  # 防止 CPU 占用过高

        except KeyboardInterrupt:
            print("Program interrupted by user")
        except (serial.SerialException, PermissionError) as e:
            print(f"Error: {e}")
            sys.exit(1)


    def stop(self):
        self.running = False


if __name__ == '__main__':
    z1_imu = Z1IMUClient(IMUPUBDATATOPIC)

    while True:

        time.sleep(0.001)
