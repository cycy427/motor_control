import serial
import time
import struct
import pandas as pd
from crcmod.predefined import mkCrcFun
import os
from datetime import datetime
import csv

last_time = None
time_intervals = []
freq_window_size = 10

# 配置串口参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际情况修改串口号
BAUD_RATE = 9600
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# 发送指令
SEND_COMMAND = b'\x81\x03\x00\x38\x00\x02\x5A\x06'


# 初始化串口
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUD_RATE,
    parity=PARITY,
    stopbits=STOP_BITS,
    bytesize=BYTESIZE,
    timeout=1
)

# CRC-16计算函数
crc16_func = mkCrcFun('modbus')

def calculate_crc(data):
    return struct.pack('<H', crc16_func(data))

def send_command(command):
    ser.write(command)
    print(f"Sent command: {command.hex()}")

def read_response():
    response = bytearray()
    while True:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            response.extend(byte)
            if len(response) >= 5 and len(response) == (response[2] + 5):  # 检查是否读取到完整响应
                break
    return response

def parse_response(response):
    addr, cmd, length = struct.unpack_from('>BBB', response[:3])
    data_length = length // 2
    registers = []
    for i in range(data_length):
        msb, lsb = struct.unpack_from('>BB', response[3 + 2 * i:5 + 2 * i])
        register_value = (msb << 8) | lsb
        registers.append(register_value)
    received_crc = struct.unpack_from('>H', response[-2:])[0]
    calculated_crc = struct.unpack('>H', calculate_crc(response[:-2]))[0]
    if received_crc != calculated_crc:
        raise ValueError("CRC check failed")
    return registers


def parse_register_data(registers):
    """解析寄存器数据"""
    parsed_data = {}

    # 电池总电压 (0x38)
    total_voltage_raw = registers[0]  # 第1个寄存器
    parsed_data["Total_Voltage"] = total_voltage_raw * 0.1

    # 电流数据 (0x39)
    current_raw = registers[1]  # 第2个寄存器
    parsed_data["Current"] = (current_raw - 30000) * 0.1

    return parsed_data


def write_parsed_data_to_csv(parsed_data, filename='processed_data_log_2.csv'):
    # 新增时间戳列（毫秒级）
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 取前3位毫秒
    parsed_data["Timestamp"] = timestamp

    # 定义字段顺序
    fieldnames = ["Timestamp","Total_Voltage", "Current"]

    # 写入 CSV
    file_exists = os.path.exists(filename)
    with open(filename, 'a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists or os.stat(filename).st_size == 0:
            writer.writeheader()
        writer.writerow(parsed_data)


try:
    while True:
        send_command(SEND_COMMAND)
        time.sleep(0.001)  # 等待0.01秒后再次发送命令
        response = read_response()
        try:
            registers = parse_response(response)
            parsed_data = parse_register_data(registers)


            # 频率计算
            current_time = time.time()
            if last_time is not None:
                interval = current_time - last_time
                time_intervals.append(interval)

                # 保留最近N个间隔值，避免无限增长
                if len(time_intervals) > freq_window_size:
                    time_intervals.pop(0)

                # 计算平均频率
                avg_interval = sum(time_intervals) / len(time_intervals)
                frequency = round(1.0 / avg_interval, 2) if avg_interval > 0 else 0.0
                parsed_data["Frequency"] = frequency
                print(f"Current frequency: {frequency} Hz")

            last_time = current_time

            # write_parsed_data_to_csv(parsed_data)
            # print(f"Processed and saved data: {parsed_data}")
            time.sleep(0.001)  # 等待0.01秒后再次发送命令
        except Exception as e:
            print(f"Error parsing response: {e}")

finally:
    ser.close()



