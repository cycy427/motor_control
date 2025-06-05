import serial
import time
import struct
import pandas as pd
from crcmod.predefined import mkCrcFun
import os
from datetime import datetime
import csv


# 配置串口参数
SERIAL_PORT = '/dev/ttyUSB0'  # 根据实际情况修改串口号
BAUD_RATE = 9600
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS

# 发送指令
SEND_COMMAND = b'\x81\x03\x00\x00\x00\x7F\x1B\xEA'

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


def parse_register_data(row):
    """解析寄存器数据"""
    parsed_data = {}

    # 单体电池电压 (0x00 ~ 0x06)，提取前7个值
    for i in range(7):  # 提取前7个寄存器的值
        register_value = int(row[f"Register_{i+1}"])
        voltage = register_value * 0.001
        parsed_data[f"Battery_{i+1}"] = voltage

    # 电池温度 (0x30 ~ 0x31)，提取前2个值
    for i in range(2):  # 提取前2个寄存器的值
        register_value = int(row[f"Register_{48+i+1}"])
        temperature = register_value - 40
        parsed_data[f"Temperature_{i+1}"] = temperature

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

    return parsed_data

def write_parsed_data_to_csv(parsed_data, filename='processed_data_log.csv'):
    # 新增时间戳列（毫秒级）
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 取前3位毫秒
    parsed_data["Timestamp"] = timestamp

    # 定义字段顺序
    fieldnames = ["Timestamp",
                  "Battery_1", "Battery_2", "Battery_3", "Battery_4", "Battery_5", "Battery_6", "Battery_7",
                  "Temperature_1", "Temperature_2",
                  "Total_Voltage", "Current", "SOC", "Max_Cell_Voltage", "Min_Cell_Voltage", "Max_Temperature",
                  "Remaining_Capacity", "Average_Voltage", "MOS_Temperature",
                  "Limit_Status", "Limit_Current", "RTC_Time"]

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
        response = read_response()
        try:
            registers = parse_response(response)
            parsed_data = parse_register_data({f"Register_{i+1}": val for i, val in enumerate(registers)})
            write_parsed_data_to_csv(parsed_data)
            print(f"Processed and saved data: {parsed_data}")
        except Exception as e:
            print(f"Error parsing response: {e}")
        time.sleep(0.01)  # 等待0.01秒后再次发送命令
finally:
    ser.close()



