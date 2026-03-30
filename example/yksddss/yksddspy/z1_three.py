# by mrtang
# 2025/4/12

# ============================================================================
# 本脚本实现了基于 Cyclone DDS 的 Z1 机器人远程控制客户端。
# 主要功能：
# 1. 定义 Z1RemoteClient 类，作为后台线程运行，用于发布电机指令和订阅电机状态。
# 2. 提供针对下肢(leg)、上肢(arm)、躯干(body)的电机控制接口。
# 3. 主程序示例：创建下肢和上肢客户端，初始化电机参数，进入循环持续发送指令。
# ============================================================================

import time
from pyexpat.errors import messages   # 导入 messages 但实际未使用（可能为误写）

from cyclonedds.domain import DomainParticipant    # DDS 域参与者
from cyclonedds.pub import Publisher, DataWriter   # DDS 发布端
from cyclonedds.sub import DataReader              # DDS 订阅端
from cyclonedds.topic import Topic                 # DDS 话题
from cyclonedds.qos import Qos, Policy             # DDS 服务质量策略
from cyclonedds.core import DDSException, Listener # DDS 异常和监听器
import nubotddsmsg.hr as hrmsg                     # 自定义 DDS 消息类型（由 IDL 生成）
from copy import deepcopy                          # 深拷贝
import threading                                   # 多线程
from cyclonedds.idl import IdlStruct, IdlUnion, IdlBitmask, IdlEnum, types  # IDL 相关（未使用）

# ============================================================================
# 常量定义：DDS 话题名称，用于发布命令和订阅状态
# ============================================================================
ARMCMDTOPIC = "/nubot/z1/armmotorcmds"      # 上肢命令话题
LEGCMDTOPIC = "/nubot/z1/legmotorcmds"      # 下肢命令话题
ARMSTATETOPIC = "/nubot/z1/armmotorstates"  # 上肢状态话题
LEGSTATETOPIC = "/nubot/z1/legmotorstates"  # 下肢状态话题
BODYCMDTOPIC = "/nubot/z1/bodymotorcmds"    # 躯干命令话题
BODYSTATETOPIC = "/nubot/z1/bodymotorstates" # 躯干状态话题

# ============================================================================
# Z1RemoteClient 类：继承 threading.Thread，作为后台线程运行
# 功能：通过 DDS 发布电机指令，订阅电机状态，并维护内部锁保证线程安全
# ============================================================================
class Z1RemoteClient(threading.Thread):
    def __init__(self, cmdtopic, statetopic, role):
        '''
        :param cmdtopic:   发布命令的 topic 名称
        :param statetopic: 订阅状态的 topic 名称
        :param role:       角色，'leg'（下肢，12个电机）、'arm'（上肢，12个电机）、'body'（躯干，6个电机）
        '''
        super(Z1RemoteClient, self).__init__()   # 调用父类初始化
        self.cmdtopic = cmdtopic
        self.statetopic = statetopic

        # 检查角色合法性
        if role not in ['arm', 'leg','body']:
            raise ValueError("role must be 'arm' or 'leg'")

        # 根据角色确定电机数量及对应的 level 编号（由消息定义决定）
        motornum = {'arm': 12, 'leg': 12, 'body': 6}
        levels = {'leg': 0, 'arm': 1, 'body': 2}
        self.daemon = True   # 设为守护线程，主线程结束时自动退出

        # --- 初始化命令消息（内部实际发送的副本）---
        # _motorCmds 是后台线程实际发送的消息对象，通过 setCommand() 从 motorCmds 深拷贝过来
        self._motorCmds = hrmsg.motorcmds(level=levels[role],
                                          cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        # motorCmds 是用户层修改的命令对象，用户通过控制接口修改它，然后调用 setCommand()
        self.motorCmds = hrmsg.motorcmds(level=levels[role],
                                         cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        # 状态消息对象，存储从 DDS 接收的最新状态
        self._motorStates = hrmsg.motorstates(level=levels[role],
                                              states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in
                                                      range(motornum[role])])

        # 创建 DDS 域参与者（域 ID 为 0）
        participant = DomainParticipant(domain_id=0)

        ###########################################################################
        # 发布端配置
        ###########################################################################
        # 创建话题（命令话题）
        cmdtopic = Topic(participant, self.cmdtopic, hrmsg.motorcmds)
        # 定义 QoS：可靠性为 BestEffort（尽力而为），持久性为 Volatile，历史记录保留最近 5 条
        cmdqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5),
        )
        publisher = Publisher(participant)          # 创建发布者
        self.writer = DataWriter(publisher, cmdtopic, qos=cmdqos)  # 创建数据写入器

        ###########################################################################
        # 订阅端配置
        ###########################################################################
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, hrmsg.motorstates, qos=stateqos)  # 状态话题
        self.reader = DataReader(participant, statetopic)  # 创建数据读取器

        # 运行标志和锁
        self.running = True
        self._lockcmd = threading.RLock()   # 保护命令消息的锁
        self._lockstate = threading.RLock() # 保护状态消息的锁

        self.start()  # 启动线程，执行 run() 方法

    # ------------------------------------------------------------------------
    # 线程主循环：以约 500Hz 的频率发布当前命令，并接收状态
    # ------------------------------------------------------------------------
    def run(self):
        while self.running:
            msgs = []
            # --- 处理订阅：从 DDS 读取状态消息 ---
            try:
                msgs = self.reader.take()   # 获取所有待处理的消息
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
            # 如果取到了消息，保留最后一条（最新状态）并更新 _motorStates
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._motorStates = msgs[-1]
                self._lockstate.release()

            # --- 处理发布：发送当前命令 ---
            self._lockcmd.acquire()
            try:
                self.writer.write(self._motorCmds)   # 将当前命令写入 DDS
            except DDSException as e:
                print("[Writer] catch DDSException error. msg:", e.msg)
            except Exception as e:
                print("[Writer] write sample error. msg:", e.args())
            except:
                print("[Writer] write sample error.")
            self._lockcmd.release()

            time.sleep(0.002)  # 2ms 延时，约 500Hz

    # ------------------------------------------------------------------------
    # 停止线程，释放资源（主程序退出时调用）
    # ------------------------------------------------------------------------
    def stop(self):
        self.running = False

    # ------------------------------------------------------------------------
    # 将用户修改的命令（motorCmds）同步到实际发送的命令（_motorCmds）
    # 需要加锁，因为后台线程同时读取 _motorCmds
    # ------------------------------------------------------------------------
    def setCommand(self):
        self._lockcmd.acquire()
        self._motorCmds = deepcopy(self.motorCmds)
        self._lockcmd.release()

    # ------------------------------------------------------------------------
    # 获取最近一次接收到的电机状态（深拷贝，避免外部修改）
    # ------------------------------------------------------------------------
    def getStates(self):
        self._lockstate.acquire()
        states = deepcopy(self._motorStates)
        self._lockstate.release()
        return states

    # ============================================================================
    # 以下为针对不同角色的电机控制接口，供用户调用
    # ============================================================================

    # --- 上肢控制（ti5 型号，只支持位置/力矩/速度单个值设定）---
    def arm_ti5_squat_control(self, arm_index, value, mode):
        """
        :param arm_index: 关节索引（全局 ID，12~23）
        :param value: 目标值（根据 mode 决定是位置、力矩或速度）
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
        """
        arm_index = arm_index - 12   # 转换为本地索引（0~11）
        if not (0 <= arm_index < len(self.motorCmds.cmds)):
            raise IndexError("Invalid motor index")
        cmd = self.motorCmds.cmds[arm_index]
        cmd.mode = mode
        if mode == 1:
            cmd.pos = value
        elif mode == 2:
            cmd.tau = value
        elif mode == 3:
            cmd.vel = value

    # --- 上肢控制（yks 型号，可同时设置位置、速度、力矩、kp、kd）---
    def arm_yks_squat_control(self, arm_index, mode, pos, vel, tau, kp, kd):
        """
        :param arm_index: 关节索引（全局 ID，12~23）
        :param mode: 控制模式 (0: 力位混合, 1: position, 2: torque, 3: velocity)
        :param pos, vel, tau, kp, kd: 控制参数
        """
        if not (12 <= arm_index < 24):
            raise IndexError("Invalid motor index")
        arm_index = arm_index - 12
        cmd = self.motorCmds.cmds[arm_index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    # --- 躯干控制（yks 型号）---
    def body_yks_squat_control(self, arm_index, mode, pos, vel, tau, kp, kd):
        """
        :param arm_index: 关节索引（全局 ID，24~29）
        :param mode, pos, vel, tau, kp, kd: 同上
        """
        if not (24 <= arm_index < 30):
            raise IndexError("Invalid motor index")
        arm_index = arm_index - 24
        cmd = self.motorCmds.cmds[arm_index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    # --- 下肢控制 ---
    def leg_squat_control(self, leg_index, mode, pos, vel, tau, kp, kd):
        """
        :param leg_index: 关节索引（全局 ID，0~11）
        :param mode, pos, vel, tau, kp, kd: 同上
        """
        if not (0 <= leg_index < 12):
            raise IndexError("Invalid motor index")
        cmd = self.motorCmds.cmds[leg_index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    # --- 读取上肢状态（ti5 型号）---
    def read_arm_ti5_control(self, arm_index, mode):
        """
        :param arm_index: 关节索引（全局 ID，12~23）
        :param mode: 读取模式 (1: position, 2: torque, 3: velocity)
        :return: 对应模式下的状态值
        """
        arm_index = arm_index - 12
        if not (0 <= arm_index < len(self._motorStates.states)):
            raise IndexError("Invalid motor index")
        state = self._motorStates.states[arm_index]
        if mode == 1:
            return state.pos
        elif mode == 2:
            return state.tau
        elif mode == 3:
            return state.vel
        else:
            return 0


# ============================================================================
# 主程序入口（演示如何使用 Z1RemoteClient）
# ============================================================================
if __name__ == '__main__':
    # 创建上肢客户端和下肢客户端（躯干客户端被注释）
    z1_arm = Z1RemoteClient(ARMCMDTOPIC, ARMSTATETOPIC, 'arm')
    z1_leg = Z1RemoteClient(LEGCMDTOPIC, LEGSTATETOPIC, 'leg')
    # z1_body = Z1RemoteClient(BODYCMDTOPIC, BODYSTATETOPIC, 'body')

    # 初始化所有下肢电机：设置 kp=500, kd=20，其他参数为 0
    for i in range(12):
        z1_leg.leg_squat_control(i, 0, 0, 0, 0, 500, 20)
        time.sleep(0.01)   # 短暂延时，避免一次发送过多

    # 预定义一些角度数组（用于后续运动，但这里未使用）
    joint_angles = [0.35, 0, 0, 0.5, 0.25, 0,
                    -0.35, 0, 0, 0.5, -0.25, 0]
    ksp_stance = [500, 500, 500, 500, 500, 500,
                  500, 500, 500, 500, 500, 500]
    ksd_stance = [50, 50, 5, 50, 20, 20,
                  50, 50, 5, 50, 20, 20]
    j = 0
    time.sleep(0.01)

    # 主循环：持续发送下肢命令，并获取状态（演示使用）
    while True:
        # 注意：这里没有修改 motorCmds，只是重复发送之前设置的值
        z1_leg.setCommand()          # 将 motorCmds 同步到实际发送的命令
        st = z1_leg.getStates()      # 获取最新状态（此处未使用，仅示例）
        # 其他被注释的代码包括：上肢控制、打印等
        time.sleep(0.1)              # 主循环每 0.1 秒执行一次