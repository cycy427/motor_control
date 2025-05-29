#!/usr/bin/env python3
# by mrtang
# 2025/4/12

import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy

import nubotddsmsg.unitree as unitreemsg
from copy import deepcopy

import threading

UNITREEPUBDATATOPIC = "/nubot/z1/unitreepubdata"

class Z1UnitreePUBClient(threading.Thread):
    def __init__(self, statetopic):
        '''
        :param statetopic: 订阅状态的topic
        '''

        super(Z1UnitreePUBClient, self).__init__()

        self.statetopic = statetopic

        self.daemon = True

        self.unitreeStates = unitreemsg.unitreedata(vx=0., vy=0.,omega=0.)

        self._unitreeStates = unitreemsg.unitreedata(vx=0., vy=0.,omega=0.)

        # 创建域参与者
        participant = DomainParticipant(0)
        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.statetopic, unitreemsg.unitreedata)
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
            self.writer.write(self._unitreeStates)
            self._lockcmd.release()
            time.sleep(0.01)  # 1000Hz

    def stop(self):
        self.running = False

    def Move(self,vx, vy,omega):
        self._lockcmd.acquire()
        self._unitreeStates.vx = vx
        self._unitreeStates.vy = vy
        self._unitreeStates.omega = omega
        self._lockcmd.release()
    def setCommand(self):
        self._lockcmd.acquire()
        self._unitreeStates = deepcopy(self.unitreeStates)
        self._lockcmd.release()

if __name__ == '__main__':

    i=0
    z1_unitree = Z1UnitreePUBClient(UNITREEPUBDATATOPIC)

    while True:
        i=i+1
        z1_unitree.Move(vx=0.1*i, vy=0.2*i, omega=0.3*i)
        # print("Move to ", z1_unitree._unitreeStates.vx, z1_unitree._unitreeStates.vy, z1_unitree._unitreeStates.omega)
        time.sleep(0.1)
