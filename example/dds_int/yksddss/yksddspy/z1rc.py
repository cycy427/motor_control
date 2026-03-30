
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
    def __init__(self,cmdtopic,statetopic,role):
        '''
        :param cmdtopic:   发布命令的topic
        :param statetopic: 订阅状态的topic
        :param role:       leg-下肢， arm-上肢
        '''

        super(Z1RemoteClient,self).__init__()

        self.cmdtopic = cmdtopic
        self.statetopic = statetopic

        if role not in ['arm','leg']:
            raise ValueError("role must be 'arm' or 'leg'")

        motornum = {'arm':14,'leg':13}
        levels = {'arm':1,'leg':0}

        self.daemon = True

        # 初始化消息
        self._motorCmds = hrmsg.motorcmds(level=levels[role],
                                         cmds=[hrmsg.motorcmd(0,0,0,0,0,0,0) for _ in range(motornum[role])])

        self.motorCmds = hrmsg.motorcmds(level=levels[role],
                                          cmds=[hrmsg.motorcmd(0,0,0,0,0,0,0) for _ in range(motornum[role])])

        self._motorStates = hrmsg.motorstates(level=levels[role],
                                             states=[hrmsg.motorstate(0,0,0,0,0,0,0,0,0,0) for _ in range(motornum[role])])

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
            Policy.Durability.Volatile,     # 或 Policy.Durability.TransientLocal
            Policy.History.KeepLast(5),    # 保留最后5条消息
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
            if len(msgs)>0:
                self._lockstate.acquire()
                self._motorStates = msgs[-1]
                self._lockstate.release()

            ## 处理发布
            self._lockcmd.acquire()
            writer.write(self._motorCmds)
            self._lockcmd.release()

            time.sleep(0.5)    # 500Hz

    def stop(self):
        self.running = False

    def setCommand(self):
        self._lockcmd.acquire()
        self._motorCmds = deepcopy(self.motorCmds)
        self._lockcmd.release()

    def getStates(self): # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._motorStates)
        self._lockstate.release()
        return states

if __name__ == '__main__':
    z1 = Z1RemoteClient(ARMCMDTOPIC,ARMSTATETOPIC,'arm')
    # z1 = Z1RemoteClient(LEGCMDTOPIC,LEGSTATETOPIC,'leg')
    for i in range(100):
        z1.motorCmds.level = i
        z1.motorCmds.cmds[0].pos = i
        z1.setCommand()
        print("pub %d"%(i))

        st = z1.getStates()
        print("sub %d"%(st.states[0].index))
        time.sleep(1)
