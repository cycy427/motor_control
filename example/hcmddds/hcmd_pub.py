#!/usr/bin/env python3
# by LUO
# 2025/4/12

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.hcmd as hcmdmsg
from copy import deepcopy

import threading

HCMDPUBDATATOPIC = "/nubot/z1/hcmdpubdata"

class Z1HcmdPUBClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1HcmdPUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self.hcmdStates = hcmdmsg.hcmddata(vx=0., vy=0.,omega=0.)

        self._hcmdStates = hcmdmsg.hcmddata(vx=0., vy=0.,omega=0.)

        # 创建域参与者
        participant = DomainParticipant(0)
        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.statetopic, hcmdmsg.hcmddata)
        cmdqos = Qos(
            Policy.Reliability.BestEffort,  # 或 Policy.Reliability.Reliable
            Policy.Durability.Volatile,  # 或 Policy.Durability.TransientLocal
            Policy.History.KeepLast(5),  # 保留最后5条消息
        )
        publisher = Publisher(participant)
        self.writer = DataWriter(publisher, cmdtopic, qos=cmdqos)
        ###########################################################################

        self.running = True
        self._lockcmd = threading.RLock()

        self.start()

    def run(self):
        while self.running:
            ## 处理发布
            self._lockcmd.acquire()
            self.writer.write(self._hcmdStates)
            self._lockcmd.release()
            time.sleep(0.01)  # 1000Hz

    def stop(self):
        self.running = False

    def Move(self,vx, vy,omega):
        self._lockcmd.acquire()
        self._hcmdStates.vx = vx
        self._hcmdStates.vy = vy
        self._hcmdStates.omega = omega
        self._lockcmd.release()
    def setCommand(self):
        self._lockcmd.acquire()
        self._hcmdStates = deepcopy(self.hcmdStates)
        self._lockcmd.release()

if __name__ == '__main__':

    i=0
    z1_hcmd = Z1HcmdPUBClient(HCMDPUBDATATOPIC)

    while True:
        i=i+1
        z1_hcmd.Move(vx=0.1*i, vy=0.2*i, omega=0.3*i)
        print(z1_hcmd._hcmdStates.vx,z1_hcmd._hcmdStates.vy,z1_hcmd._hcmdStates.omega)
        time.sleep(0.1)
