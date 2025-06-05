# by luo
# 2025/4/12

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

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

        """
        :param Battery_Voltage: 电池电压*7 Max_Cell_Voltage: 最大电池电压 Min_Cell_Voltage: 最小电池电压 Average_Voltage: 平均电压
        :param Temperature : 电池温度*2 Max_Temperature：最大温度 MOS_Temperature：MOS管温度
        :param Total_Voltage: 总电压
        :param Current: 电流
        :param SOC: 剩余容量
        :param Remaining_Capacity: 剩余容量
        :param Limit_Status: 限流状态
        :param Limit_Current: 限流电流
        :param RTC_Time: 实时时间
        """

        self._BmsStates =bmsmsg.bmsdata(Battery_Voltage=[0.] * 10, Temperature=[0.] * 4, Total_Voltage=0., Current=0.,
                                        SOC=0., Remaining_Capacity=0., Limit_Status='', Limit_Current=0., RTC_Time='')

        # 创建域参与者
        participant = DomainParticipant(0)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, bmsmsg.bmsdata, qos=stateqos)
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
                self._BmsStates = msgs[-1]
                self._lockstate.release()

            time.sleep(0.02)  # 100Hz

    def stop(self):
        self.running = False

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
