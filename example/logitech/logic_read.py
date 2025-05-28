#!/usr/bin/env python3
import evdev
from evdev import ecodes

# 寻找游戏手柄设备
def find_gamepad():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        # Logitech F710 可能显示为 "Logitech Gamepad F710"
        if "Logitech" in device.name:
            print(f"找到设备: {device.path} ({device.name})")
            return device
    raise OSError("未找到游戏手柄设备")

# 映射按钮名称
BUTTON_MAP = {
    ecodes.BTN_SOUTH:    'A',        # 按钮A（下方按钮）
    ecodes.BTN_EAST:     'B',        # 按钮B（右侧按钮）
    ecodes.BTN_NORTH:    'X',        # 按钮X（上方按钮）
    ecodes.BTN_WEST:     'Y',        # 按钮Y（左侧按钮）
    ecodes.BTN_TL:       'LB',       # 左肩按钮
    ecodes.BTN_TR:       'RB',       # 右肩按钮
    ecodes.BTN_SELECT:   'BACK',
    ecodes.BTN_START:    'START',
    ecodes.BTN_MODE:     'HOME' ,      # 菜单按钮
    ecodes.BTN_THUMBL:   'LP',       # 左摇杆按下
    ecodes.BTN_THUMBR:   'RP',       # 右摇杆按下
}

# 摇杆轴映射
AXIS_MAP = {
    0: 'LEFT_X',        # 左摇杆X轴
    1: 'LEFT_Y',        # 左摇杆Y轴
    2: 'LT',       # 右摇杆X轴
    3: 'RIGHT_X',       # 右摇杆Y轴
    4: 'RIGHT_Y',
    5: 'RT',
    16: 'DPAD_X',       # 方向键 X 轴（圆盘）
    17: 'DPAD_Y'        # 方向键 Y 轴（圆盘）
}

def main():
    gamepad = find_gamepad()
    print("正在监听游戏手柄输入，按 Ctrl+C 退出...")

    # 读取事件循环
    for event in gamepad.read_loop():
        # 处理按键事件
        if event.type == ecodes.EV_KEY:
            # 过滤未定义的按钮
            if event.code in BUTTON_MAP:
                button = BUTTON_MAP[event.code]
                state = "按下" if event.value else "释放"
                print(f"按钮 {button}: {state}")

        # 处理摇杆事件
        elif event.type == ecodes.EV_ABS:
            axis = AXIS_MAP.get(event.code, f"未知轴{event.code}")
            # 将原始值转换为标准化值 (-1.0 到 1.0)
            value = event.value / 32767.0 if event.value > 0 else event.value / 32768.0
            print(f"摇杆 {axis}: {value:.3f}")

if __name__ == "__main__":
    try:
        main()
    except OSError as e:
        print(e)
    except PermissionError:
        print("权限不足，请尝试使用sudo运行或配置udev规则")