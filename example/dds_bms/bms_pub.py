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


class Z1BMSPUBClient(threading.Thread):
    def __init__(self, statetopic, port='/dev/ttyACM0', baudrate=9600):
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
        self.SEND_COMMAND = b'\x81\x03\x00\x00\x00\x7F\x1B\xEA'

        # CRC计算函数
        self.crc16_func = mkCrcFun('modbus')
        # CSV输出文件
        self.output_file = 'processed_data_log.csv'
        """
        :param Battery_Voltage: 电池电压*12 Max_Cell_Voltage: 最大电池电压 Min_Cell_Voltage: 最小电池电压 Average_Voltage: 平均电压
        :param Temperature : 电池温度*5 Max_Temperature：最大温度 MOS_Temperature：MOS管温度
        :param Total_Voltage: 总电压
        :param Current: 电流
        :param SOC: 剩余容量
        :param Remaining_Capacity: 剩余容量
        :param Limit_Status: 限流状态
        :param Limit_Current: 限流电流
        :param RTC_Time: 实时时间
        """
        self.BmsStates = bmsmsg.bmsdata(timestamp='', Battery_Voltage=[0.] * 15, Temperature=[0.] * 7, Total_Voltage=0.,
                                        Current=0.,
                                        SOC=0., Remaining_Capacity=0., Limit_Status='', Limit_Current=0., RTC_Time='')

        self._BmsStates = bmsmsg.bmsdata(timestamp='', Battery_Voltage=[0.] * 15, Temperature=[0.] * 7,
                                         Total_Voltage=0., Current=0.,
                                         SOC=0., Remaining_Capacity=0., Limit_Status='', Limit_Current=0., RTC_Time='')

        # 创建域参与者
        participant = DomainParticipant(0)
        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.statetopic, bmsmsg.bmsdata)
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

        # self.start()

    def run(self):
        try:
            while self.running:
                self.send_command()
                time.sleep(0.01)  # 增加等待时间，让设备有时间响应
                response = self.read_response()
                try:
                    registers = self.parse_response(response)
                    parsed_data = self.parse_register_data(
                        {f"Register_{i + 1}": val for i, val in enumerate(registers)}
                    )

                    self.update_bms_states(parsed_data)  # 发布
                    print(f"Processed and saved data: {parsed_data}")
                    time.sleep(0.01)  # 增加等待时间，让设备有时间响应

                except Exception as e:
                    print(f"Error parsing response: {e}")
        finally:
            self.ser.close()

    def stop(self):
        self.running = False
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

            elif time.time() - start_time > 0.5:  # 设置超时避免死循环
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

    def parse_register_data(self, row):
        """解析寄存器数据"""
        parsed_data = {}

        # 单体电池电压 (0x00 ~ 0x06)，提取前12个值
        for i in range(12):  # 提取前12个寄存器的值
            register_value = int(row[f"Register_{i + 1}"])
            voltage = register_value * 0.001
            parsed_data[f"Battery_{i + 1}"] = voltage

        # 电池温度 (0x30 ~ 0x31)，提取前5个值
        for i in range(5):  # 提取前5个寄存器的值
            register_value = int(row[f"Register_{48 + i + 1}"])
            temperature = register_value - 40
            parsed_data[f"Temperature_{i + 1}"] = temperature

        # 电池总电压 (0x38)
        total_voltage_raw = int(row["Register_57"])  # Register_57 对应地址 0x38
        parsed_data["Total_Voltage"] = total_voltage_raw * 0.1

        # 电流数据 (0x39)
        current_raw = int(row["Register_58"])  # Register_58 对应地址 0x39
        parsed_data["Current"] = (current_raw - 30000) * 0.1

        # SOC (0x3A)
        soc_raw = int(row["Register_59"])  # Register_59 对应地址 0x3A
        parsed_data["SOC"] = soc_raw * 0.001

        # 最高单体电压 (0x3E)
        max_cell_voltage_raw = int(row["Register_63"])  # Register_63 对应地址 0x3E
        parsed_data["Max_Cell_Voltage"] = max_cell_voltage_raw / 1000.0

        # 最低单体电压 (0x40)
        min_cell_voltage_raw = int(row["Register_65"])  # Register_65 对应地址 0x40
        parsed_data["Min_Cell_Voltage"] = min_cell_voltage_raw / 1000.0

        # 最高单体温度 (0x43)
        max_temp_raw = int(row["Register_68"])  # Register_68 对应地址 0x43
        parsed_data["Max_Temperature"] = max_temp_raw - 40

        # 剩余容量 (0x4B)
        remaining_capacity_raw = int(row["Register_76"])  # Register_76 对应地址 0x4B
        parsed_data["Remaining_Capacity"] = remaining_capacity_raw * 0.1

        # 平均电压 (0x57)
        avg_voltage_raw = int(row["Register_88"])  # Register_88 对应地址 0x57
        parsed_data["Average_Voltage"] = avg_voltage_raw / 1000.0

        # MOS 温度 (0x5A)
        mos_temp_raw = int(row["Register_91"])  # Register_91 对应地址 0x5A
        parsed_data["MOS_Temperature"] = mos_temp_raw - 40

        # 限流状态 (0x5F)
        limit_status_raw = int(row["Register_96"])  # Register_96 对应地址 0x5F
        parsed_data["Limit_Status"] = "Enabled" if limit_status_raw == 1 else "Disabled"

        # 限流电流 (0x60)
        limit_current_raw = int(row["Register_97"])  # Register_97 对应地址 0x60
        parsed_data["Limit_Current"] = (limit_current_raw - 30000) * 0.1

        # RTC 时间 (0x61 ~ 0x63)
        rtc_year_month_day = int(row["Register_98"])  # Register_98 对应地址 0x61
        rtc_hour_minute_second = int(row["Register_99"])  # Register_99 对应地址 0x62
        year = (rtc_year_month_day >> 8) & 0xFF
        month = (rtc_year_month_day >> 4) & 0xF
        day = rtc_year_month_day & 0xF
        hour = (rtc_hour_minute_second >> 8) & 0xFF
        minute = (rtc_hour_minute_second >> 4) & 0xF
        second = rtc_hour_minute_second & 0xF
        parsed_data["RTC_Time"] = f"{year:02d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d}"
        #添加时间戳
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 毫秒级时间戳
        parsed_data["Timestamp"] = timestamp
        return parsed_data

    def write_parsed_data_to_csv(self, parsed_data):
        """写入解析后的数据到CSV"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 毫秒级时间戳
        parsed_data["Timestamp"] = timestamp

        fieldnames = ["Timestamp",
                      "Battery_1", "Battery_2", "Battery_3", "Battery_4", "Battery_5", "Battery_6", "Battery_7",
                      "Temperature_1", "Temperature_2",
                      "Total_Voltage", "Current", "SOC", "Max_Cell_Voltage", "Min_Cell_Voltage", "Max_Temperature",
                      "Remaining_Capacity", "Average_Voltage", "MOS_Temperature",
                      "Limit_Status", "Limit_Current", "RTC_Time"]

        file_exists = os.path.exists(self.output_file)

        with open(self.output_file, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists or os.stat(self.output_file).st_size == 0:
                writer.writeheader()
            writer.writerow(parsed_data)

    def update_bms_states(self, parsed_data):
        with self._lockcmd:  # 使用锁保护共享资源
            bms_states = self.BmsStates

            # 电池电压 (Battery_1 ~ Battery_7): 原始12个单体电压
            battery_voltages = [parsed_data[f"Battery_{i + 1}"] for i in range(12)]

            # 添加 Max_Cell_Voltage, Min_Cell_Voltage, Average_Voltage 到数组后三位
            battery_voltages += [
                parsed_data["Max_Cell_Voltage"],
                parsed_data["Min_Cell_Voltage"],
                parsed_data["Average_Voltage"]
            ]
            bms_states.Battery_Voltage = battery_voltages  # 长度应为 15

            # 温度传感器 (Temperature_1 ~ Temperature_2): 原始5个温度传感器数据
            temperatures = [parsed_data[f"Temperature_{i + 1}"] for i in range(5)]

            # 添加 Max_Temperature, MOS_Temperature 到数组后两位
            temperatures += [
                parsed_data["Max_Temperature"],
                parsed_data["MOS_Temperature"]
            ]
            bms_states.Temperature = temperatures  # 长度应为 7

            # 其他字段保持不变
            bms_states.timestamp = parsed_data["Timestamp"]
            bms_states.Total_Voltage = parsed_data["Total_Voltage"]
            bms_states.Current = parsed_data["Current"]
            bms_states.SOC = parsed_data["SOC"]
            bms_states.Remaining_Capacity = parsed_data["Remaining_Capacity"]
            bms_states.Limit_Status = parsed_data["Limit_Status"]
            bms_states.Limit_Current = parsed_data["Limit_Current"]
            bms_states.RTC_Time = parsed_data["RTC_Time"]

            # 更新状态
            self._BmsStates = bms_states

            # 发布消息
            self.writer.write(self._BmsStates)


if __name__ == '__main__':
    z1_bms = Z1BMSPUBClient(BMSPUBDATATOPIC)
    z1_bms.run()

    # while True:
    #
    #     time.sleep(0.01)
