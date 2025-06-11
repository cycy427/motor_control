#!/usr/bin/env python3
# by mrtang
# 2025/4/12

import time
import threading

from channel import ChannelSubscriber, ChannelFactoryInitialize
import nubotddsmsg.hcmd as hcmdmsg
from copy import deepcopy


HCMDPUBDATATOPIC = "/nubot/z1/hcmdsubdata"


class Z1HcmdSUBClient(threading.Thread):
    def __init__(self, statetopic):
        super(Z1HcmdSUBClient, self).__init__()

        # 初始化域（domain_id=0）
        ChannelFactoryInitialize(id=0)

        # 初始化订阅器
        self.subscriber = ChannelSubscriber(statetopic, hcmdmsg.hcmddata, handler=self.__OnDataAvailable)
        self.subscriber.Init(queueLen=10)  # 启用队列缓存最多10条消息

        # 当前命令状态
        self._hcmdStates = hcmdmsg.hcmddata(vx=0., vy=0., omega=0.)
        self.reader_valid = False

        self.running = True
        self._lockstate = threading.RLock()

        self.start()

    def run(self):
        while self.running:
            time.sleep(0.01)  # 保持主线程活跃，定期检查状态等操作

    def stop(self):
        self.running = False
        self.subscriber.Close()  # 关闭订阅通道

    def __OnDataAvailable(self, sample):
        """
        DataReader 回调函数，当有新数据到达时自动触发
        """
        if isinstance(sample, InvalidSample):
            self.reader_valid = False
            return

        self._lockstate.acquire()
        try:
            self._hcmdStates = sample
            self.reader_valid = True
        finally:
            self._lockstate.release()

    def getStates(self):
        """
        获取当前最新的控制指令
        """
        self._lockstate.acquire()
        try:
            return deepcopy(self._hcmdStates)
        finally:
            self._lockstate.release()


if __name__ == '__main__':
    z1_hcmd = Z1HcmdSUBClient(HCMDPUBDATATOPIC)

    while True:
        st = z1_hcmd.getStates()
        print("sub :", st)

        time.sleep(0.1)
