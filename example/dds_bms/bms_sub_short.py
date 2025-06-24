# by luo
# 2025/4/12

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.core import DDSException, Listener

import nubotddsmsg.bms as bmsmsg
from copy import deepcopy

import threading

BMSPUBDATATOPIC = "/nubot/z1/bmspubdata"


class Z1BMmsSUBClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1BMmsSUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self._BmsStates =bmsmsg.bmsdata_short(all_voltage=0.,all_current=0.)

        # 创建域参与者
        participant = DomainParticipant(0)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, bmsmsg.bmsdata_short, qos=stateqos)
        self.reader = DataReader(participant, statetopic)
        ###########################################################################

        self.running = True
        self.reader_valid = False

        self._lockstate = threading.RLock()
        self.start()

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
                self._BmsStates = msgs[-1]
                self._lockstate.release()

            time.sleep(0.02)  # 100Hz

    def stop(self):
        self.running = False
        if self.reader is not None:
            del self.reader
    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._BmsStates)
        self._lockstate.release()
        return states


if __name__ == '__main__':
    z1_bms = Z1BMmsSUBClient(BMSPUBDATATOPIC)

    while True:
        st = z1_bms.getStates()
        print("sub :" ,st)

        time.sleep(0.02)
