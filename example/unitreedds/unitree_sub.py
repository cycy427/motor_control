# by mrtang
# 2025/4/12

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.unitree as unitreemsg
from copy import deepcopy

import threading

UNITREEPUBDATATOPIC = "/nubot/z1/unitreepubdata"


class Z1UnitreeSUBClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1UnitreeSUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self._unitreeStates = unitreemsg.unitreedata(vx=0., vy=0.,omega=0.)
        # 创建域参与者
        participant = DomainParticipant(0)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, unitreemsg.unitreedata, qos=stateqos)
        self.reader = DataReader(participant, statetopic)
        ###########################################################################

        self.running = True
        self._lockstate = threading.RLock()
        self.start()

    def run(self):
        while self.running:
            ## 处理订阅
            msgs = self.reader.take()
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._unitreeStates = msgs[-1]
                self._lockstate.release()

            time.sleep(0.01)  # 100Hz

    def stop(self):
        self.running = False

    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._unitreeStates)
        self._lockstate.release()
        return states


if __name__ == '__main__':
    z1_unitree = Z1UnitreeSUBClient(UNITREEPUBDATATOPIC)

    while True:
        st = z1_unitree.getStates()
        print("sub :" ,st)

        time.sleep(0.1)
