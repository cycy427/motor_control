# by mrtang
# 2025/4/12

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.logic as logicmsg
from copy import deepcopy

import threading

LOGICPUBDATATOPIC = "/nubot/z1/logicpubdata"


class Z1LogicSUBClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1LogicSUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self._logicStates = logicmsg.logicdata(button_map=[0.]*11,axes_map=[0.]*8)
        # 创建域参与者
        participant = DomainParticipant(0)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.Reliable(1000),
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, logicmsg.logicdata, qos=stateqos)
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
                self._logicStates = msgs[-1]
                self._lockstate.release()

            time.sleep(0.02)  # 100Hz

    def stop(self):
        self.running = False

    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._logicStates)
        self._lockstate.release()
        return states


if __name__ == '__main__':
    z1_logic = Z1LogicSUBClient(LOGICPUBDATATOPIC)

    while True:
        st = z1_logic.getStates()
        print("sub :" ,st)

        time.sleep(0.01)
