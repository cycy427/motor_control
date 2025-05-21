# by mrtang
# 2025/4/12

import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.idl.types import sequence
import nubotddsmsg.hr as hrmsg
from copy import deepcopy

import threading

ARMCMDTOPIC = "/nubot/z1/armmotorcmds"
LEGCMDTOPIC = "/nubot/z1/legmotorcmds"

ARMSTATETOPIC = "/nubot/z1/armmotorstates"
LEGSTATETOPIC = "/nubot/z1/legmotorstates"

BODYCMDTOPIC = "/nubot/z1/bodymotorcmds"
BODYSTATETOPIC = "/nubot/z1/bodymotorstates"

WHOLEBODYCMDTOPIC = "/nubot/z1/wholebodymotorcmds"
WHOLEBODYSTATETOPIC = "/nubot/z1/wholebodymotorstates"
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

        if role not in ['arm', 'leg', 'body', 'Z1_5_WB']:
            raise ValueError("role must be 'arm' or 'leg' or 'body' or 'Z1_5_WB'")

        motornum = {'arm': 12, 'leg': 12, 'body': 6, 'Z1_5_WB': 30}
        levels = {'leg': 0, 'arm': 1, 'body': 2, 'Z1_5_WB': 3}

        self.daemon = True

        # 初始化消息
        self._motorCmds = hrmsg.motorcmds(level=levels[role],
                                          cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        # MotorCmdSeq = sequence(hrmsg.motorcmd)
        # cmd_list = [hrmsg.motorcmd(mode=0, index=i, pos=0.0, vel=0.0, tau=0.0, kp=0.0, kd=0.0) for i in range(12)]
        # cmds = sequence(hrmsg.motorcmd, cmd_list)  # 或者 MotorCmdSeq(cmd_list)
        # cmds = MotorCmdSeq
        #
        # self._motorCmds = hrmsg.motorcmds(level=levels[role],
        #                                   cmds=MotorCmdSeq)

        self.motorCmds = hrmsg.motorcmds(level=levels[role],
                                         cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        self._motorStates = hrmsg.motorstates(level=levels[role],
                                              states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in
                                                      range(motornum[role])])


        # 创建域参与者
        participant = DomainParticipant(0)

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

    def run(self):

        while self.running:
            ## 处理订阅
            msgs = self.reader.take()
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._motorStates = msgs[-1]
                self._lockstate.release()

            ## 处理发布
            self._lockcmd.acquire()
            self.writer.write(self._motorCmds)
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
        arm_index = arm_index - 11
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
        arm_index = arm_index - 12

        if not (0 <= arm_index < len(self.motorCmds.cmds)):
            raise IndexError("Invalid motor index")

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
        arm_index = arm_index - 24

        if not (0 <= arm_index < len(self.motorCmds.cmds)):
            raise IndexError("Invalid motor index")

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
        if not (0 <= leg_index < len(self.motorCmds.cmds)):
            raise IndexError("Invalid motor index")

        cmd = self.motorCmds.cmds[leg_index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    def z1_5_wb_squat_control(self, index, mode, pos, vel, tau, kp, kd):
        """
        :param index: 关节索引（全局ID）双足：0-11 双臂 12-23 躯干 24-29
        :param pos, vel, tau, kp, kd: 目标值
        :param mode: 控制模式 (0: 力位混合 1: position, 2: torque, 3: velocity)
        """
        if not (0 <= index < len(self.motorCmds.cmds)):
            raise IndexError("Invalid motor index")

        cmd = self.motorCmds.cmds[index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd
    def read_arm_control(self, arm_index, mode):
        """
        :param arm_index: 关节索引（全局ID）
        :param mode: 读取模式 (0: 力位混合 1: position, 2: torque, 3: velocity)
        :return: 对应模式下的状态值
        """
        arm_index = arm_index - 11  # 转换为本地索引
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


if __name__ == '__main__':
    # z1_arm = Z1RemoteClient(ARMCMDTOPIC, ARMSTATETOPIC, 'arm')
    z1_5_wb = Z1RemoteClient(WHOLEBODYCMDTOPIC, WHOLEBODYSTATETOPIC, 'Z1_5_WB')
    # z1_body = Z1RemoteClient(BODYCMDTOPIC, BODYSTATETOPIC, 'body')

    # z1_5_wb.leg_squat_control(22, 0, 1, 0, 0, 400, 40)#0-11
    
    for i in range(30):
        # print(i)
        z1_5_wb.z1_5_wb_squat_control(i, 0, 0, 0, 0, 0, 10)

        time.sleep(0.01)


        
    while True:
        z1_5_wb.setCommand()
        print("pub %f  %f" % (0, z1_5_wb.motorCmds.cmds[0].pos))

        st = z1_5_wb.getStates()
        print("sub %f" % (st.states[0].pos))
        # z1_body.setCommand()
        # print("pub %f  %f" % (0, z1_leg.motorCmds.cmds[0].pos))

        time.sleep(0.01)
