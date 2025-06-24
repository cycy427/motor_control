# by mrtang
# 2025/4/12

import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.core import DDSException, Listener

import nubotddsmsg.sensor as sensormsg
from copy import deepcopy

import threading

IMUSUBDATATOPIC = "/nubot/z1/imusubdata"
IMUPUBDATATOPIC = "/nubot/z1/imupubdata"


class Z1IMUClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1IMUClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self._imuStates = sensormsg.imudata(orientation=sensormsg.orientation(x=0, y=0, z=0, w=0),
                                            angular_velocity=sensormsg.angular_velocity(x=0, y=0, z=0),
                                            linear_acceleration=sensormsg.linear_acceleration(x=0, y=0, z=0),
                                            Magnetic=sensormsg.Magnetic(x=0, y=0, z=0),
                                            euler_angles=sensormsg.euler_angles(roll=0, pitch=0, yaw=0))

        self.running = True
        self.reader_valid = False

        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()

        # 创建域参与者
        participant = DomainParticipant(0)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, sensormsg.imudata, qos=stateqos)
        self.reader = DataReader(participant, statetopic)
        self.start()

    def run(self):


        while self.running:
            msgs = []
            ## 处理订阅
            try:
                msgs = self.reader.take()
                if not msgs:
                    self.reader_valid = False
                else:
                    self.reader_valid = True
            except DDSException as e:
                print("[Reader] catch DDSException msg:", e.msg)
            except TimeoutError as e:
                print("[Reader] take sample timeout")
            except:
                print("[Reader] take sample error")
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._imuStates = msgs[-1]
                self._lockstate.release()

            time.sleep(0.001)  # 1000Hz

    def stop(self):
        self.running = False
        if self.reader is not None:
            del self.reader
    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._imuStates)
        self._lockstate.release()
        return states

    def read_arm_control(self):

        state = self._imuStates.angular_velocity.x
        return state


if __name__ == '__main__':
    z1_imu = Z1IMUClient(IMUPUBDATATOPIC)

    # z1_leg.leg_squat_control(12, 0, 2, 0, 0, 400, 40)

    while True:
        st = z1_imu.getStates()
        # print("sub %f" % (st.angular_velocity.x))
        print("sub :" ,st)
        # print("sub %f" % st.angular_velocity.x)
        # print(f"Position: {pos}, Torque: {torque}, Velocity: {vel}")
        # time.sleep(0.01)
