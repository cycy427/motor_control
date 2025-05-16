import sys
import time
import serial
from utils import clear_screen
from parsers.hipnuc_serial_parser import hipnuc_parser
from parsers.hipnuc_nmea_parser import hipnuc_nmea_parser


def read_data():
    # 设置串口号和波特率
    port = '/dev/ttyUSB0'  # 修改为你实际使用的串口号
    baudrate = 921600     # 修改为你需要的波特率

    serial_parser = hipnuc_parser()
    nmea_parser = hipnuc_nmea_parser()

    frame_count = 0
    frame_rate = 0
    last_frame_time = time.time()
    last_display_time = time.time()
    display_interval = 0.2  # 每0.2秒更新一次显示

    latest_hipnuc_frame = None
    latest_nmea_frames = []

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"Connected to {port} at {baudrate} baud.")
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)

                    try:
                        hipnuc_frames = serial_parser.parse(data)
                        nmea_frames = nmea_parser.parse(data.decode('ascii', errors='ignore'))

                        frame_count += len(hipnuc_frames) + len(nmea_frames)

                        if hipnuc_frames:
                            latest_hipnuc_frame = hipnuc_frames[-1]
                        if nmea_frames:
                            latest_nmea_frames = nmea_frames

                        current_time = time.time()
                        if current_time - last_frame_time >= 1.0:
                            frame_rate = frame_count
                            frame_count = 0
                            last_frame_time = current_time

                        # 定期更新显示
                        if current_time - last_display_time >= display_interval:
                            clear_screen()

                            if latest_hipnuc_frame:
                                serial_parser.print_parsed_data(latest_hipnuc_frame)
                            if latest_nmea_frames:
                                nmea_parser.print_parsed_data(latest_nmea_frames)

                            print(f"Frame rate: {frame_rate} Hz")
                            last_display_time = current_time

                    except Exception as e:
                        print(f"Error parsing data: {e}")

                time.sleep(0.001)  # 防止 CPU 占用过高

    except KeyboardInterrupt:
        print("Program interrupted by user")
    except (serial.SerialException, PermissionError) as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    read_data()
