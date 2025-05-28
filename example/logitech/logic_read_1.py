#!/usr/bin/env python3
import evdev
from evdev import ecodes

# 寻找游戏手柄设备
def find_gamepad():
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if "Logitech" in device.name:
            print(f"找到设备: {device.path} ({device.name})")
            return device
    raise OSError("未找到游戏手柄设备")

# 映射按钮名称
BUTTON_MAP = {
    ecodes.BTN_SOUTH:    'A',        # 按钮A（下方按钮）
    ecodes.BTN_EAST:     'B',        # 按钮B（右侧按钮）
    ecodes.BTN_NORTH:    'Y',        # 按钮Y（上方按钮）
    ecodes.BTN_WEST:     'X',        # 按钮X（左侧按钮）
    ecodes.BTN_START:    'START',
    ecodes.BTN_SELECT:   'BACK',
    ecodes.BTN_TR:       'RB',       # 右肩按钮
    ecodes.BTN_TL:       'LB',       # 左肩按钮
    ecodes.BTN_THUMBR:   'R3',       # 右摇杆按下
    ecodes.BTN_THUMBL:   'L3',       # 左摇杆按下
    ecodes.BTN_TL2:      'LP',       # 左下肩按钮（模拟）
    ecodes.BTN_TR2:      'RP',       # 右下肩按钮（模拟）
}

# 摇杆轴映射
AXIS_MAP = {
    0: 'LEFT_X',        # 左摇杆X轴
    1: 'LEFT_Y',        # 左摇杆Y轴
    2: 'RIGHT_X',       # 右摇杆X轴
    3: 'RIGHT_Y',       # 右摇杆Y轴
    16: 'DPAD_X',       # 方向键 X 轴（圆盘）
    17: 'DPAD_Y',       # 方向键 Y 轴（圆盘）
}

def main():
    gamepad = find_gamepad()
    print("正在监听游戏手柄输入，按 Ctrl+C 退出...")

    # 初始化按键状态字典
    button_states = {name: 0.0 for name in BUTTON_MAP.values()}

    # 初始化摇杆状态字典
    axis_states = {name: 0.0 for name in AXIS_MAP.values()}
    axis_states.update({
        'LEFT_PRESS': 0.0,
        'RIGHT_PRESS': 0.0,
    })

    # 读取事件循环
    for event in gamepad.read_loop():
        # 处理按键事件
        if event.type == ecodes.EV_KEY and event.code in BUTTON_MAP:
            button = BUTTON_MAP[event.code]
            axis_states[button] = float(event.value)  # 0.0 或 1.0

        # 处理摇杆事件
        elif event.type == ecodes.EV_ABS:
            axis_name = AXIS_MAP.get(event.code, None)
            if axis_name:
                value = event.value
                normalized_value = value / 32767.0 if value > 0 else value / 32768.0
                axis_states[axis_name] = normalized_value

        # 构造按键列表（顺序：A,B,X,Y,LB,RB,Back,Start,HOME,LP,RP）
        button_list = [
            axis_states['A'],
            axis_states['B'],
            axis_states['X'],
            axis_states['Y'],
            axis_states['LB'],
            axis_states['RB'],
            axis_states['SELECT'],
            axis_states['START'],
            axis_states['HOME'],
            axis_states['LP'],
            axis_states['RP'],
        ]

        # 构造摇杆列表（顺序：左边左右、左边前后、左边按下，右边左右、右边前后、右边按下，圆盘左右，圆盘前后）
        axis_list = [
            axis_states['LEFT_X'],
            axis_states['LEFT_Y'],
            axis_states['L3'],
            axis_states['RIGHT_X'],
            axis_states['RIGHT_Y'],
            axis_states['R3'],
            axis_states['DPAD_X'],
            axis_states['DPAD_Y'],
        ]

        # 打印结果
        print(f"按钮列表: {[round(x, 3) for x in button_list]}")
        print(f"摇杆列表: {[round(x, 3) for x in axis_list]}")

if __name__ == "__main__":
    try:
        main()
    except OSError as e:
        print(e)
    except PermissionError:
        print("权限不足，请尝试使用sudo运行或配置udev规则")
