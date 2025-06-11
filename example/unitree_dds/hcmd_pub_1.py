#!/usr/bin/env python3
# by LUO
# 2025/4/12

import time
import threading

from channel import ChannelPublisher, ChannelFactoryInitialize
import nubotddsmsg.hcmd as hcmdmsg
from copy import deepcopy


HCMDPUBDATATOPIC = "/nubot/z1/hcmdsubdata"


class Z1HcmdPUBClient(threading.Thread):
    def __init__(self, statetopic):
        super(Z1HcmdPUBClient, self).__init__()

        # 初始化域（domain_id=0）
        ChannelFactoryInitialize(id=0)

        # 初始化发布器
        self.publisher = ChannelPublisher(statetopic, hcmdmsg.hcmddata)
        self.publisher.Init()

        # 当前命令状态
        self.hcmdStates = hcmdmsg.hcmddata(vx=0., vy=0., omega=0.)
        self._hcmdStates = hcmdmsg.hcmddata(vx=0., vy=0., omega=0.)

        self.running = True
        self._lockcmd = threading.RLock()

        self.start()

    def run(self):
        while self.running:
            self._lockcmd.acquire()
            try:
                # 发送当前命令
                success = self.publisher.Write(self._hcmdStates)
                if not success:
                    print("[Z1HcmdPUBClient] Failed to write command.")
            except Exception as e:
                print(f"[Z1HcmdPUBClient] Write error: {e}")
            finally:
                self._lockcmd.release()

            time.sleep(0.01)  # 1000Hz

    def stop(self):
        self.running = False
        self.publisher.Close()  # 关闭发布通道

    def Move(self, vx, vy, omega):
        self._lockcmd.acquire()
        try:
            self._hcmdStates.vx = vx
            self._hcmdStates.vy = vy
            self._hcmdStates.omega = omega
        finally:
            self._lockcmd.release()

    def setCommand(self):
        self._lockcmd.acquire()
        try:
            self._hcmdStates = deepcopy(self.hcmdStates)
        finally:
            self._lockcmd.release()


if __name__ == '__main__':
    i = 0
    z1_hcmd = Z1HcmdPUBClient(HCMDPUBDATATOPIC)

    while True:
        i += 1
        z1_hcmd.Move(vx=0.1 * i, vy=0.2 * i, omega=0.3 * i)
        print(z1_hcmd._hcmdStates.vx, z1_hcmd._hcmdStates.vy, z1_hcmd._hcmdStates.omega)
        time.sleep(0.1)
