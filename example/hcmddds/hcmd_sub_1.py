# by mrtang
# 2025/4/12

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.hcmd as hcmdmsg
from copy import deepcopy

import threading

HCMDPUBDATATOPIC = "/nubot/z1/hcmdpubdata"


class Z1HcmdSUBClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1HcmdSUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self._hcmdStates = hcmdmsg.hcmddata(vx=0., vy=0.,omega=0.)

        self.last_valid_time = time.time()  # 记录最后一次收到有效数据的时间

        # 创建域参与者
        participant = DomainParticipant(0)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5),
            # Policy.Ownership.Exclusive,
            Policy.Liveliness.Automatic(lease_duration=1)
        )
        statetopic = Topic(participant, self.statetopic, hcmdmsg.hcmddata, qos=stateqos)
        self.reader = DataReader(participant, statetopic)
        ###########################################################################

        self.running = True
        self._lockstate = threading.RLock()
        self.start()

    def run(self):

        valid_received = False
        while self.running:
            ## 处理订阅
            msgs = self.reader.take()
            print(f"收到 {msgs} 个数据包")
            for msg in msgs:
                if not msg.info.valid_data:
                    print("收到无效样本，可能发布者已离线")
                    continue

                timestamp = msg.info.timestamp  # 使用 DDS 自带时间戳
                print(f"[{timestamp}] 收到有效数据: {msg}")
                valid_received = True

            if valid_received:
                self._lockstate.acquire()
                self._hcmdStates = msgs[-1]
                self.last_valid_time = time.time()  # 更新最后收到时间
                self._lockstate.release()

            # 检查是否超过1秒未收到有效数据
            if time.time() - self.last_valid_time > 1.0:
                print("[警告] 超过1秒未收到有效的hcmd数据！")

            time.sleep(0.01)  # 100Hz

    def stop(self):
        self.running = False

    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._hcmdStates)
        self._lockstate.release()
        return states


if __name__ == '__main__':
    z1_hcmd = Z1HcmdSUBClient(HCMDPUBDATATOPIC)

    while True:
        st = z1_hcmd.getStates()
        print("sub :" ,st)

        time.sleep(0.1)
