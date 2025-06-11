#!/usr/bin/env python3
# by luo
# 2025/4/12

import serial
import time
import struct
from crcmod.predefined import mkCrcFun
import os
from datetime import datetime
import csv

import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.bms as bmsmsg
from copy import deepcopy

import threading

BMSPUBDATATOPIC = "/nubot/z1/bmspubdata"
INTERVAL = 0.0001 #  发送指令的间隔时间（单位：秒）
DURATION = 1  # 采集时间（单位：秒）

class Z1BMSPUBClient(threading.Thread):
    def __init__(self, statetopic, port='/dev/ttyUSB0', baudrate=9600):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1BMSPUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        # 串口配置
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        # 发送指令
        self.SEND_COMMAND = b'\x81\x03\x00\x38\x00\x02\x5A\x06'

        self.last_time = None  # 上次接收到数据的时间
        self.time_intervals = []  # 存储时间间隔用于平滑处理
        self.freq_window_size = 10  # 使用最近10个间隔计算平均频率

        # CRC计算函数
        self.crc16_func = mkCrcFun('modbus')
        # CSV输出文件
        # self.start_time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        # self.output_file = f'/tmp/bmslog/bms_data_{self.start_time_str}.csv'
        self.output_file = f'/tmp/bmslog/bms_data_.csv'

        self.BmsStates = bmsmsg.bmsdata_short(all_voltage=0.,all_current=0.)
        self._BmsStates = bmsmsg.bmsdata_short(all_voltage=0.,all_current=0.)

        # 创建域参与者
        participant = DomainParticipant(0)
        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.statetopic, bmsmsg.bmsdata_short)
        cmdqos = Qos(
            Policy.Reliability.BestEffort,  # 或 Policy.Reliability.Reliable
            Policy.Durability.Volatile,  # 或 Policy.Durability.TransientLocal
            Policy.History.KeepLast(5),  # 保留最后5条消息
        )
        publisher = Publisher(participant)
        self.writer = DataWriter(publisher, cmdtopic, qos=cmdqos)
        ###########################################################################

        self.running = True
        self._lockcmd = threading.RLock()

        self.recent_data = []  # 用于缓存最近5分钟的数据
        self.data_interval = INTERVAL
        self.cache_duration = DURATION
        self.max_cache_size = int(self.cache_duration / self.data_interval)

        self.start()

    def run(self):
        try:
            while self.running:
                self.send_command()
                time.sleep(INTERVAL)  # 增加等待时间，让设备有时间响应
                response = self.read_response()
                try:
                    registers = self.parse_response(response)
                    parsed_data = self.parse_register_data(registers)

                    # 测量频率
                    current_time = time.time()
                    if self.last_time is not None:
                        interval = current_time - self.last_time
                        self.time_intervals.append(interval)

                        # 保留最近N个间隔值，避免无限增长
                        if len(self.time_intervals) > self.freq_window_size:
                            self.time_intervals.pop(0)

                        # 计算平均频率
                        avg_interval = sum(self.time_intervals) / len(self.time_intervals)
                        frequency = 1.0 / avg_interval if avg_interval > 0 else 0.0
                        print(f"Frequency:  {round(frequency, 2)}")  # Hz

                    self.last_time = current_time
                    # 保留最新的5分钟数据
                    self.recent_data.append(parsed_data)
                    if len(self.recent_data) > self.max_cache_size:
                        self.recent_data.pop(0)  # 删除最早的数据

                    self.update_bms_states(parsed_data)  # 发布
                    print(f"Processed and saved data: {parsed_data}")

                    time.sleep(0.0001)  # 增加等待时间，让设备有时间响应

                except Exception as e:
                    print(f"Error parsing response: {e}")
        finally:
            self.ser.close()

    def stop(self):
        self.running = False
        # 写入最近5分钟的数据到CSV
        if self.recent_data:
            print("正在写入最近5分钟的数据到CSV...")
            for data in self.recent_data:
                self.write_parsed_data_to_csv(data)

        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.close()
            print("Serial closed.")

    def calculate_crc(self, data):
        return struct.pack('<H', self.crc16_func(data))

    def send_command(self):
        self.ser.reset_input_buffer()  # 添加这一行
        self.ser.write(self.SEND_COMMAND)
        print(f"Sent command: {self.SEND_COMMAND.hex()}")

    def read_response(self):
        self.ser.reset_input_buffer()  # 清除输入缓冲区
        response = bytearray()
        start_time = time.time()
        while True:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                response.extend(byte)

                # 判断是否是 Modbus RTU 响应帧（至少3字节头 + 数据长度 + CRC）
                if len(response) >= 3:
                    length = response[2]  # 第三个字节表示数据长度
                    expected_length = 3 + length + 2  # 数据长度 + CRC两个字节
                    if len(response) == expected_length:
                        break

            elif time.time() - start_time > 2:  # 设置超时避免死循环
                raise TimeoutError("Response timeout")
        return response

    def parse_response(self, response):
        addr, cmd, length = struct.unpack_from('>BBB', response[:3])
        data_length = length // 2
        registers = []
        for i in range(data_length):
            msb, lsb = struct.unpack_from('>BB', response[3 + 2 * i:5 + 2 * i])
            register_value = (msb << 8) | lsb
            registers.append(register_value)
        received_crc = struct.unpack_from('>H', response[-2:])[0]
        calculated_crc = struct.unpack('>H', self.calculate_crc(response[:-2]))[0]
        if received_crc != calculated_crc:
            raise ValueError("CRC check failed")
        return registers

    def parse_register_data(self, registers):
        """解析寄存器数据"""
        parsed_data = {}
        # 电池总电压 (0x38)
        total_voltage_raw = registers[0]  # 第1个寄存器
        parsed_data["Total_Voltage"] = total_voltage_raw * 0.1

        # 电流数据 (0x39)
        current_raw = registers[1]  # 第2个寄存器
        parsed_data["Current"] = (current_raw - 30000) * 0.1
        #添加时间戳
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 毫秒级时间戳
        parsed_data["Timestamp"] = timestamp
        return parsed_data
    def write_parsed_data_to_csv(self, parsed_data):
        # 创建目录（如果不存在）
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        """写入解析后的数据到CSV"""
        fieldnames = ["Timestamp","Total_Voltage", "Current"]

        file_exists = os.path.exists(self.output_file)

        with open(self.output_file, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists or os.stat(self.output_file).st_size == 0:
                writer.writeheader()
            writer.writerow(parsed_data)

    def update_bms_states(self, parsed_data):
        with self._lockcmd:  # 使用锁保护共享资源
            bms_states = self.BmsStates
            # 其他字段保持不变
            bms_states.all_voltage= parsed_data["Total_Voltage"]
            bms_states.all_current = parsed_data["Current"]

            # 更新状态
            self._BmsStates = bms_states

            # 发布消息
            self.writer.write(self._BmsStates)

if __name__ == '__main__':
    z1_bms = Z1BMSPUBClient(BMSPUBDATATOPIC)
    # z1_bms.run()
    try:
        # 等待用户输入以停止程序
        input("按 Enter 键退出...\n")
    finally:
        z1_bms.stop()
print("程序已退出。")

    # while True:
    #
    #     time.sleep(0.01)
