# by mrtang
# 2025/4/12

import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.core import DDSException, Listener

import nubotddsmsg.hr as hrmsg
from copy import deepcopy

import threading

from cyclonedds.idl import IdlStruct, IdlUnion, IdlBitmask, IdlEnum, types


ARMCMDTOPIC = "/nubot/z1/armmotorcmds"
LEGCMDTOPIC = "/nubot/z1/legmotorcmds"

ARMSTATETOPIC = "/nubot/z1/armmotorstates"
LEGSTATETOPIC = "/nubot/z1/legmotorstates"

BODYCMDTOPIC = "/nubot/z1/bodymotorcmds"
BODYSTATETOPIC = "/nubot/z1/bodymotorstates"

class Z1RemoteClient(threading.Thread):
    def __init__(self, cmdtopic, statetopic, role):
        '''
        :param cmdtopic:   发布命令的topic
        :param statetopic: 订阅状态的topic
        :param role:       leg-下肢， arm-上肢 ，body-躯干
        '''

        super(Z1RemoteClient, self).__init__()

        self.cmdtopic = cmdtopic
        self.statetopic = statetopic

        if role not in ['arm', 'leg','body']:
            raise ValueError("role must be 'arm' or 'leg'")

        motornum = {'arm': 12, 'leg': 12, 'body': 6}
        levels = {'leg': 0, 'arm': 1, 'body': 2}

        self.daemon = True

        # 初始化消息
        self._motorCmds = hrmsg.motorcmds(level=levels[role],
                                          cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        self.motorCmds = hrmsg.motorcmds(level=levels[role],
                                         cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        self._motorStates = hrmsg.motorstates(level=levels[role],
                                              states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in
                                                      range(motornum[role])])

        participant = DomainParticipant(domain_id=0)

        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.cmdtopic, hrmsg.motorcmds)
        cmdqos = Qos(
            Policy.Reliability.BestEffort,  # 或 Policy.Reliability.Reliable
            Policy.Durability.Volatile,  # 或 Policy.Durability.TransientLocal
            Policy.History.KeepLast(5),  # 保留最后5条消息
        )
        publisher = Publisher(participant)
        self.writer = DataWriter(publisher, cmdtopic, qos=cmdqos)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, hrmsg.motorstates, qos=stateqos)
        self.reader = DataReader(participant, statetopic)

        self.running = True
        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()

        self.start()
        # time.sleep(0.1)



    def run(self):

        while self.running:
            msgs = []
            ## 处理订阅
            try:
                msgs = self.reader.take()
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
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._motorStates = msgs[-1]
                self._lockstate.release()

            ## 处理发布
            self._lockcmd.acquire()
            try:
                self.writer.write(self._motorCmds)
            except DDSException as e:
                print("[Writer] catch DDSException error. msg:", e.msg)
            except Exception as e:
                print("[Writer] write sample error. msg:", e.args())
            except:
                print("[Writer] write sample error.")

            self._lockcmd.release()

            time.sleep(0.002)  # 500Hz

    def stop(self):
        self.running = False

    def setCommand(self):
        self._lockcmd.acquire()
        self._motorCmds = deepcopy(self.motorCmds)
        self._lockcmd.release()

    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._motorStates)
        self._lockstate.release()
        return states


    def arm_ti5_squat_control(self, arm_index, value, mode):
        """
            :param arm_index: 关节索引
            :param value: 目标值
            :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
        """
        arm_index = arm_index - 12
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

    def arm_yks_squat_control(self, arm_index, mode, pos, vel, tau, kp, kd):
        """
        :param arm_index: 关节索引
        :param value: 目标值
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
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
    def body_yks_squat_control(self, arm_index, mode, pos, vel, tau, kp, kd):
        """
        :param arm_index: 关节索引
        :param value: 目标值
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
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

    def leg_squat_control(self, leg_index, mode, pos, vel, tau, kp, kd):
        """
        :param leg_index: 关节索引
        :param value: 目标值
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
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

    def read_arm_ti5_control(self, arm_index, mode):
        """
        :param arm_index: 关节索引（全局ID）
        :param mode: 读取模式 (0: 力位混合 1: position, 2: torque, 3: velocity)
        :return: 对应模式下的状态值
        """
        arm_index = arm_index - 12  # 转换为本地索引
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



# if __name__ == '__main__':


#     z1_leg = Z1RemoteClient(LEGCMDTOPIC, LEGSTATETOPIC, 'leg')

   
#     joint_angles = [0.35, 0, 0, 0.5, 0.25, 0,
#                     -0.35, 0, 0, 0.5, -0.25, 0]
#     ksp_stance = [500, 500, 500, 500, 500, 500,
#                   500, 500, 500, 500, 500, 500]
#     ksd_stance = [50, 50, 5, 50, 20, 20,
#                   50, 50, 5, 50, 20, 20]
#     # j = 0
#     # time.sleep(0.01)

#     j = 0
#     time.sleep(0.01)
#     # z1_body.body_yks_squat_control(24, 0, 1, 0, 0, 100, 10)#24-26

#     while True:
#         # z1_arm.arm_yks_squat_control(23, 0, 1, 0, 0, 500, 10)#12-23

#         for i in range(12):
#             z1_leg.leg_squat_control(i, 0, joint_angles[i]*j*0.01, 0, 0, 500, ksd_stance[i])
#         # z1.squat_control(11, 0.5, 1)

#         # z1_arm.setCommand()
#         # print("pub %f  %f" % (13, z1_arm.motorCmds.cmds[6].pos))

#         # st = z1_arm.getStates()
#         # print(st.states)

#         z1_leg.setCommand()
#         # print("pub %f  %f" % (0, z1_leg.motorCmds.cmds[0].pos))

#         st = z1_leg.getStates()
#         # print(st.states)
#         # z1_body.setCommand()
#         # print("pub %f  %f" % (0, z1_leg.motorCmds.cmds[0].pos))

#         # st = z1_body.getStates()
#         # print(st.states)
#         j += 1
#         if j >=  100:
#             j = 100
#         print("j" ,j)
#         time.sleep(0.01)
if __name__ == '__main__':
    # 初始化机械腿客户端
    z1_leg = Z1RemoteClient(LEGCMDTOPIC, LEGSTATETOPIC, 'leg')
    time.sleep(0.5)  # 短时间等待连接

    # 目标角度
    target_angles = [0.35, 0, 0, 0.5, 0.25, 0,
                     -0.35, 0, 0, 0.5, -0.25, 0]
    # 刚度和阻尼参数
    ksp_stance = [500] * 12
    ksd_stance = [50, 50, 5, 50, 20, 20,
                  50, 50, 5, 50, 20, 20]

    # 获取初始角度（关键：从当前位置开始）
    initial_angles = [0.0] * 12
    while True:
        st = z1_leg.getStates()
        if st.states and len(st.states) >= 12:
            for i in range(12):
                initial_angles[i] = st.states[i].pos  # 读取当前角度
            break
        time.sleep(0.01)
    print("初始角度:", [round(a, 4) for a in initial_angles])

    # 过渡参数（减少步数，加快过渡）
    total_steps = 50  # 总步数（比之前少，过渡更快）
    current_step = 0

    # 过渡主循环
    while current_step <= total_steps:
        # 计算当前步目标角度（从初始到目标的线性过渡）
        current_targets = [
            initial_angles[i] + (target_angles[i] - initial_angles[i]) * (current_step / total_steps)
            for i in range(12)
        ]

        # 发送指令
        for i in range(12):
            z1_leg.leg_squat_control(i, 0, current_targets[i], 0, 0, ksp_stance[i], ksd_stance[i])
        z1_leg.setCommand()

        # 进度更新
        current_step += 1
        print(f"步骤 {current_step}/{total_steps}")
        time.sleep(0.02)  # 稍长间隔，进一步加快整体过渡

    # 到达目标后保持
    print("已到达目标位置，保持状态")
    while True:
        for i in range(12):
            z1_leg.leg_squat_control(i, 0, target_angles[i], 0, 0, ksp_stance[i], ksd_stance[i])
        z1_leg.setCommand()
        time.sleep(0.05)