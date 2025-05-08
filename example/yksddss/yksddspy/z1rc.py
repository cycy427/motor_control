# by mrtang
# 2025/4/12

import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.hr as hrmsg
from copy import deepcopy

import threading

ARMCMDTOPIC = "/nubot/z1/armmotorcmds"
LEGCMDTOPIC = "/nubot/z1/legmotorcmds"

ARMSTATETOPIC = "/nubot/z1/armmotorstates"
LEGSTATETOPIC = "/nubot/z1/legmotorstates"


class Z1RemoteClient(threading.Thread):
    def __init__(self, cmdtopic, statetopic, role):
        '''
        :param cmdtopic:   发布命令的topic
        :param statetopic: 订阅状态的topic
        :param role:       leg-下肢， arm-上肢
        '''

        super(Z1RemoteClient, self).__init__()

        self.cmdtopic = cmdtopic
        self.statetopic = statetopic

        if role not in ['arm', 'leg']:
            raise ValueError("role must be 'arm' or 'leg'")

        motornum = {'arm': 14, 'leg': 13}
        levels = {'arm': 1, 'leg': 0}

        self.daemon = True

        # 初始化消息
        self._motorCmds = hrmsg.motorcmds(level=levels[role],
                                          cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        self.motorCmds = hrmsg.motorcmds(level=levels[role],
                                         cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        self._motorStates = hrmsg.motorstates(level=levels[role],
                                              states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in
                                                      range(motornum[role])])

        self.running = True
        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()
        self.start()

    def run(self):
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
        writer = DataWriter(publisher, cmdtopic, qos=cmdqos)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, hrmsg.motorstates, qos=stateqos)
        reader = DataReader(participant, statetopic)

        while self.running:
            ## 处理订阅
            msgs = reader.take()
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._motorStates = msgs[-1]
                self._lockstate.release()

            ## 处理发布
            self._lockcmd.acquire()
            writer.write(self._motorCmds)
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
    def squat_control(self, arm_index, value, mode):
        """
        :param arm_index: 关节索引
        :param value: 目标值
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
        """
        arm_index  = arm_index - 12
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

    def read_arm_control(self, arm_index, mode):
        """
        :param arm_index: 关节索引（全局ID）
        :param mode: 读取模式 (1: position, 2: torque, 3: velocity)
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


#     /读取电机状态，根据电机全局id进行索引，通过模式确认返回值
# //
# //mode 1 : position control 单位 弧度
# //mode 2 : torque control 单位 Nm 没有测试过，根据文档计算得出，可以直接给电流
# //mode 3 : velocity control 单位 rad/s
# double read_arm_control( int arm_index,int mode ) {
#     switch (mode) {
#     case 1: //position
# return my_motor_data[arm_index].pos_; //
#
# case 2: //torque
# return  my_motor_data[arm_index].tau_; //
#
# case 3: //velocity
# return my_motor_data[arm_index].vel_; //
# default:
# return 0;
# }
# }




if __name__ == '__main__':
    z1 = Z1RemoteClient(ARMCMDTOPIC, ARMSTATETOPIC, 'arm')
    # z1 = Z1RemoteClient(LEGCMDTOPIC, LEGSTATETOPIC, 'leg')

    z1.squat_control(13, 2, 3)
    # z1.squat_control(14, 50, 2)

    while True:
        # z1.squat_control(11, 0.5, 1)

        z1.setCommand()
        print("pub %f  %f" % (13, z1.motorCmds.cmds[1].pos))

        st = z1.getStates()
        print("sub %f" % (st.states[1].pos))

        pos = z1.read_arm_control(13, 1)  # 获取第13个关节的位置
        torque = z1.read_arm_control(13, 2)  # 获取力矩
        vel = z1.read_arm_control(13, 3)  # 获取速度
        print(f"Position: {pos}, Torque: {torque}, Velocity: {vel}")
        time.sleep(0.01)
