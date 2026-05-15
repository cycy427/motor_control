import struct
from crcmod.predefined import mkCrcFun

# 现有命令帧（不含 CRC-16 校验码）
data = b'\x81\x03\x00\x38\x00\x02'

# CRC-16 计算函数（Modbus 协议）
crc16_func = mkCrcFun('modbus')

# 计算 CRC-16 校验码
crc_value = crc16_func(data)

# 将校验码转换为两个字节（低字节在前，高字节在后）
crc_bytes = struct.pack('<H', crc_value)

# 构建完整命令帧
complete_command = data + crc_bytes

# 打印完整命令帧（十六进制格式）
print(f"Complete command: {complete_command.hex()}")
