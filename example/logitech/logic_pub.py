#!/usr/bin/env python3
# by mrtang
# 2025/4/12

import evdev
from evdev import ecodes

import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.logic as logicmsg
from copy import deepcopy

import threading

LOGICPUBDATATOPIC = "/nubot/z1/logicpubdata"

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
class Z1LogicPUBClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1LogicPUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self.logicStates = logicmsg.logicdata(button_map=[0.]*11,axes_map=[0.]*8)

        self._logicStates = logicmsg.logicdata(button_map=[0.]*11,axes_map=[0.]*8)

        # 创建域参与者
        participant = DomainParticipant(0)
        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.statetopic, logicmsg.logicdata)
        cmdqos = Qos(
            Policy.Reliability.Reliable(1000),  # 或 Policy.Reliability.Reliable
            Policy.Durability.Volatile,  # 或 Policy.Durability.TransientLocal
            Policy.History.KeepLast(5),  # 保留最后5条消息
        )
        publisher = Publisher(participant)
        self.writer = DataWriter(publisher, cmdtopic, qos=cmdqos)
        ###########################################################################
        try:
            self.gamepad = self.find_gamepad()
            print("正在监听游戏手柄输入，按 Ctrl+C 退出...")
        except OSError as e:
            print(e)
        except PermissionError:
            print("权限不足，请尝试使用sudo运行或配置udev规则")


        self.running = True
        self._lockcmd = threading.RLock()

        # self.start()

    def run(self):
        # 读取事件循环
        for event in self.gamepad.read_loop():
            # 处理按键事件
            if event.type == ecodes.EV_KEY:
                # 过滤未定义的按钮
                if event.code in BUTTON_MAP:
                    index = list(BUTTON_MAP.keys()).index(event.code)
                    self._logicStates.button_map[index] = 1.0 if event.value else 0.0
                    # button = BUTTON_MAP[event.code]
                    # state = "按下" if event.value else "释放"
                    # print(f"按钮 {button}: {state}")

            # 处理摇杆事件
            elif event.type == ecodes.EV_ABS:
                if event.code in AXIS_MAP:
                    axis_name = AXIS_MAP[event.code]
                    # 根据轴名称映射到 axesMap 索引
                    axis_index = {
                        'LEFT_X': 0,
                        'LEFT_Y': 1,
                        'LT': 2,
                        'RIGHT_X': 3,
                        'RIGHT_Y': 4,
                        'RT': 5,
                        'DPAD_X': 6,
                        'DPAD_Y': 7,
                    }.get(axis_name, -1)
                    if axis_index >= 0:
                        # 标准化为 [-1.0, 1.0]
                        value = event.value / 32767.0 if event.value > 0 else event.value / 32768.0
                        self._logicStates.axes_map[axis_index] = value

                # axis = AXIS_MAP.get(event.code, f"未知轴{event.code}")
                # # 将原始值转换为标准化值 (-1.0 到 1.0)
                # value = event.value / 32767.0 if event.value > 0 else event.value / 32768.0
                # print(f"摇杆 {axis}: {value:.3f}")

            ## 处理发布
            self._lockcmd.acquire()
            self.writer.write(self._logicStates)
            self._lockcmd.release()
            time.sleep(0.01)  # 1000Hz

    def stop(self):
        self.running = False

    def find_gamepad(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            # Logitech F710 可能显示为 "Logitech Gamepad F710"
            if "Logitech" in device.name:
                print(f"找到设备: {device.path} ({device.name})")
                return device
        raise OSError("未找到游戏手柄设备")


if __name__ == '__main__':
    z1_logic = Z1LogicPUBClient(LOGICPUBDATATOPIC)
    z1_logic.run()

    # while True:
    #
    #     time.sleep(0.01)
