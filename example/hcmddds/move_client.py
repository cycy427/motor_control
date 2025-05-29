import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import time
import sys
import struct

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

# Uncomment the following two lines when using Go2、Go2-W、B2、B2-W、H1 robot
# from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

# Uncomment the following two lines when using G1、H1-2 robot
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

class unitreeRemoteController:
    def __init__(self):
        # key
        self.Lx = 0           
        self.Rx = 0            
        self.Ry = 0            
        self.Ly = 0

        # button
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.Up = 0
        self.Down = 0
        self.Left = 0
        self.Right = 0
        self.Select = 0
        self.F1 = 0
        self.F3 = 0
        self.Start = 0
       
    def parse_botton(self,data1,data2):
        self.R1 = (data1 >> 0) & 1
        self.L1 = (data1 >> 1) & 1
        self.Start = (data1 >> 2) & 1
        self.Select = (data1 >> 3) & 1
        self.R2 = (data1 >> 4) & 1
        self.L2 = (data1 >> 5) & 1
        self.F1 = (data1 >> 6) & 1
        self.F3 = (data1 >> 7) & 1
        self.A = (data2 >> 0) & 1
        self.B = (data2 >> 1) & 1
        self.X = (data2 >> 2) & 1
        self.Y = (data2 >> 3) & 1
        self.Up = (data2 >> 4) & 1
        self.Right = (data2 >> 5) & 1
        self.Down = (data2 >> 6) & 1
        self.Left = (data2 >> 7) & 1

    def parse_key(self,data):
        lx_offset = 4
        self.Lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
        rx_offset = 8
        self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
        ry_offset = 12
        self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
        L2_offset = 16
        L2 = struct.unpack('<f', data[L2_offset:L2_offset + 4])[0] # Placeholder，unused
        ly_offset = 20
        self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]


    def parse(self,remoteData):
        self.parse_key(remoteData)
        self.parse_botton(remoteData[2],remoteData[3])

        # print("debug unitreeRemoteController: ")
        # print("Lx:", self.Lx)
        # print("Rx:", self.Rx)
        # print("Ry:", self.Ry)
        # print("Ly:", self.Ly)

        # print("L1:", self.L1)
        # print("L2:", self.L2)
        # print("R1:", self.R1)
        # print("R2:", self.R2)
        # print("A:", self.A)
        # print("B:", self.B)
        # print("X:", self.X)
        # print("Y:", self.Y)
        # print("Up:", self.Up)
        # print("Down:", self.Down)
        # print("Left:", self.Left)
        # print("Right:", self.Right)
        # print("Select:", self.Select)
        # print("F1:", self.F1)
        # print("F3:", self.F3)
        # print("Start:", self.Start)
        # print("\n")

    
class CmdToRpc(Node):
    def __init__(self):
        super().__init__("cmd_to_rpc")
        self.cmdsub = self.create_subscription(
            Twist,
            '/red_standard_robot1/cmd_vel',
            self.getCmdVel,
            10
        )

        network = "enp7s0"
        ChannelFactoryInitialize(0, network)
        
        self.sport_client = LocoClient()  
        self.sport_client.Init()
        # self.sport_client.Start()

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        # self.sport_client.Move(0.3,0,0)

        self.low_state = None 
        self.remoteController = unitreeRemoteController()
        self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        timer_period = 0.05

        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.control = True



    def getCmdVel(self,data):
        self.vx = data.linear.x
        self.vy = data.linear.y
        # self.vy = 0.0

        if self.vx < -0.5:
            self.vx = -0.5
        elif self.vx > 0.5:
            self.vx = 0.5
        
        # if self.vy < -0.5:
        #     self.vy = -0.5
        # elif self.vy > 0.5:
        #     self.vy = 0.
        
        # self.vy = 0.0
        
        self.omega = data.angular.z

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        wireless_remote_data = self.low_state.wireless_remote
        self.remoteController.parse(wireless_remote_data)

    def timer_callback(self):
        # self.sport_client.Move(-0.1,0.0,0.3)

        # self.sport_client.Move(0.3,0,0)
        
        # print(self.remoteController.L1)
        if self.remoteController.L1 == 1 and self.remoteController.Up == 1:
            self.control = False
        if self.remoteController.R1 == 1 and self.remoteController.X == 1:
            self.control = True
        if self.control:
           
            self.sport_client.Move(self.vx,self.vy,self.omega)
            print(self.vx,self.vy,self.omega)
        else:
            self.sport_client.Move(0,0,0)
            pass

def main(args=None):
    rclpy.init(args=args)
    cmd_to_rpc = CmdToRpc()
    rclpy.spin(cmd_to_rpc)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
    




