# by mrtang
# 2025/4/12

import time
import cyclonedds.idl.types as types

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.sbus as sbusmsg
from copy import deepcopy

import threading

SBUSSTATETOPIC = "/nubot/z1/wholebodymotorstates"



class Z1SBUSClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1SBUSClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self._subsStates = sbusmsg.sbusdata(lost_frame=False,failsafe=False,ch=[0]*16)

        self.running = True
        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()
        self.start()

    def run(self):
        # 创建域参与者
        participant = DomainParticipant(0)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, sbusmsg.sbusdata, qos=stateqos)
        reader = DataReader(participant, statetopic)

        while self.running:
            ## 处理订阅
            msgs = reader.take()
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._subsStates = msgs[-1]
                self._lockstate.release()

            time.sleep(0.001)  # 1000Hz

    def stop(self):
        self.running = False

    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._subsStates)
        self._lockstate.release()
        return states


if __name__ == '__main__':
    z1_susb = Z1SBUSClient(SBUSSTATETOPIC)

    # z1_leg.leg_squat_control(12, 0, 2, 0, 0, 400, 40)

    while True:
        st = z1_susb.getStates()
        # print("sub %f" % (st.angular_velocity.x))
        print("sub :" ,st)
        # print("sub %f" % st.angular_velocity.x)
        # print(f"Position: {pos}, Torque: {torque}, Velocity: {vel}")
        # time.sleep(0.01)
