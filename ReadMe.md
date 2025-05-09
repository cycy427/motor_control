---
author: GoldenPhilosophy
date: 2025-03-11
---

## YKS_SDK 简介

YKS_SDK是一个开源的Ethercat驱动程序，基于SOEM库实现了Ethercat主站和从站的通信，并提供了一系列的API接口，方便用户使用。

## YKS_SDK核心库： [transmit.cpp](app/transmit.cpp)

### 核心数据类型：

```
typedef struct {
 float pos_, vel_, tau_;
 float pos_des_, vel_des_, kp_, kd_, ff_;
} YKSMotorData;
```

该结构体描述了电极的位置、速度等等信息，当作为指令下发时，为期望的电机参数。当作为状态读取时，为电机的实际参数。

### 核心函数：

#### void CAT_Init(char *device_name);

- device_name为主站网卡名称，可通过ifconfig命令确定ethercat总线主站设备连接对应的网卡。

- 该函数运行后，会自动连接主站，并扫描从站，并完成通信校验等。

- 该函数成功运行后，会在后台创建线程，自动以1000Hz对从站设备收发消息。

#### void User_Get_Motor_Data(YKSMotorData *mot_data)

- 读取总线上所有电机数据，通过mot_data结构体数组返回

- 该函数为非阻塞函数，立即返回

#### void EtherCAT_Send_Command(const YKSMotorData *mot_data) （事实上我建议这个函数改名为  User_Set_Motor_Data）

- 设置电机数据

- 该函数为非阻塞函数，立即返回

#### 建议添加一个CAT_stop函数，增加停止功能

#### 建议添加一个反馈状态的函数，用于监测通信状态

### 基本流程示例（伪代码）：

    #include ...
    int main(){
        struct YKSMotorData mData[13];
        CAT_Init('eth_0'); //初始化并开启总线通讯
        for(int i=0;i<1000;i++){
            User_Get_Motor_Data(mData); //读取电机数据
            //do something
            EtherCAT_Send_Command(mData); //控制电机
            sleep...
        }
        return 0;
    }    

## YDS核心库：[Z1Legs.cpp](Z1Legs.cpp)类

使用方法见后

### 如何部署 使用root权限运行

1. 安装Readline库，该库用于从终端读取用户输入  
   以Ubuntu为例，可以使用`sudo apt install libreadline-dev`指令安装此库
   cmake 报错：Could NOT find Boost (missing: Boost_INCLUDE_DIR)；
   请安装：Boost 输入`sudo apt-get install libboost-all-dev`
2. 安装ncurses库，该库用于显示终端界面`sudo apt-get install libncurses5-dev libncursesw5-dev`
3. 使用`ifconfig`指令确定接入从机的网卡名称  `sudo apt-get install net-tools`
4. 将该网卡名称填入到[main.cpp](main.cpp)中、"在while函数中需添加不少于10MS的延时，否则电机无法正常运行"
5. 修改[transmit.h](app/transmit.h)中的最大从机数目（默认为5）
6. 如果需要使用SBUS接收机，需要修改串口的别名，才能找到这个接收机，具体使用教程可以参见 [SBUS转USB串口配置教程](https://www.wolai.com/kUuBkzjtbkCvuwPxWN3Epj)

7. 安装依赖库：
   `sudo apt install libtinfo-dev libreadline-dev libboost-all-dev libncurses5-dev libncursesw5-dev net-tools`
8. 安装DDS核心库，进入到[cyclonedds](cyclonedds)目录，执行
   `mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_EXAMPLES=ON .. && cmake --build . --parallel && sudo cmake --build . --target install`
   如果这一步报错，请删除build文件夹，重新创建，再次执行cmake命令
9. 安装DDS C++库，进入到[cyclonedds-cxx](cyclonedds-cxx)目录，执行
   `mkdir build && cd build && cmake   -DCMAKE_PREFIX_PATH=/usr/local   -DCMAKE_INSTALL_PREFIX=/usr/local   -DBUILD_EXAMPLES=ON ..
 && cmake --build . --parallel && sudo cmake --build . --target install`如果这一步报错，请删除build文件夹，重新创建，再次执行cmake命令
10. 编译本工程：
    ```shell
       mkdir build
       cd build
       cmake .. //这一步如果提示错误,请删除build文件夹,重新创建
       make
       ```
11. 启动程序
     ```shell
    sudo ./YKS_SDK
    ```
    由于soem使用到了原始套接字，该程序必须以root权限运行，也可以使用setcap为本程序单独赋予原始套接字权限，
    可以参考[这篇文章](https://squidarth.com/networking/systems/rc/2018/05/28/using-raw-sockets.html)

### 如何使用

1. 对于Z1用户
   只需要直接使用Z1Legs类，即可控制电机运动，所使用的函数是类中的public函数

   ```cpp
   #include "Z1Legs.h"
   YKSMotorData my_motor_data[Z1_NUM_MOTOR];
   int main() {
       CAT_Init("enp5s0"); //填入网卡名称
       Z1Legs z1_legs;
       float pos = 0;
       while (true) {
           if (z1_legs.stop_) {
               break;
           }
           my_motor_data[2].pos_des_ = pos; //设置电机目标位置
           z1_legs.setMotorCommand(my_motor_data); //设置电机指令
           z1_legs.getMotorData(my_motor_data); //获取电机数据
           // printf("pos: %f\n", my_motor_data[0].pos_);
       }
       runThread.join(); //runThread.join(); 的主要功能是确保 main 函数在退出之前等待 runThread 线程完成其任务。
       // 这样做的目的是为了确保程序在退出前所有的后台任务都得到了正确的执行和清理，避免数据丢失或资源泄露。
       return 0;
   }

   ```

2. 对于创建新的机器人类
   所使用的是transmit.cpp文件，该文件中包含了Ethercat主站和从站的通信协议，用户只需调用该文件中的函数即可实现自己的机器人类。

3. 对于Python用户
   这里通过Socket通信创建了一个接口，分别是example/scripts下面的[SocketReceiver.py](example/scripts/SocketReceiver.py)、[SocketSender.py](example/scripts/SocketSender.py)
   这两个文件，他们分别展示了Python与C++SDK的控制发送与数据接收方式，通过Socket通信，可以实现Python控制电机运动，并拿到电机回传的数据。

4. 对于DDS用户
   这里通过DDS通信创建了一个接口，详情参见[ReadMe.md](example/yksddss/ReadMe.md)
   ，代码在[yksddspy](example/yksddss/yksddspy)

### 特别注意

Z1Legs类，***默认为PR模式***，也就是已经经过了闭链运动学的解算，而不是直接控制单电机，是耦合控制双电机

在[motor_control.c](app/motor_control.c)和[motor_control.h](app/motor_control.h)
包含了有关Ethercat板子所接电机型号的设置，以及对应的参数设置，需要使用者提前注意设置好
在[transmit.cpp](app/transmit.cpp)中，包含所接Ti5电机和YKS电机数量的设置，以及最大从站数量的设置，需要使用者根据自己的电机数量进行修改

ethercat 从站默认分配六个通道，CAN1通道为1，2，3；CAN2通道为 4，5，6；
更改ID，等设置指令均使用CAN1，即1，2，3通道
设置速度、力矩等指令可在函数中自主选择，通道设置为函数中第二个变量“passage”可选择。

Ethercat驱动板内部有6个通道，使用的时候指定通道即可，电机的ID号必须是1-6也只能是1-6

***无论是左腿还是右腿，只要是接在Ethercat板子上面的电机，都必须是1-6的ID号，否则只能控制电机但是无法拿到电机回传的数据***

安装xone手柄驱动：（获取力反馈，并非必须）需要同时安装xone和xpadneo才能用，就听神奇的
https://gitcode.com/gh_mirrors/xo/xone

更多教程参见app文件夹下的[YKS官方教程](app/README.md#SOEM主站)

### 钛虎电机使用简要说明，请先阅读上述完整教程
C++使用方法：
1. 将build文件夹删除，重新创建工程后编译
    ```shell
       mkdir build
       cd build
       cmake .. //这一步如果提示错误,请删除build文件夹,重新创建
       make
       sudo ./YKS_SDK
2. 检查app/motor_control.c文件中的电机型号设置是否正确，如有需要，修改，电机的型号是否正确
   也就是检查Z1_MOTOR_ID_Type和g_slaves数组初始化是否正确，前者是电机的顺序
3. 在main.cpp中，检查CAT_Init("enp3s0")中的网口名称是否正确，如有需要，修改
4. 可以调用函数squat_control()和read_arm_control()来控制和读取电机数据
 这两个函数需要根据需要进行修改和拓展，比如输入数组来一次性控制多个电机，输出数组来一次性读取多个电机数据
这两个函数的电机id号参考app/motor_control.c中的g_slaves的global_id，与上位机显示的序列差1。
#### python控制电机运动：
1. 完成c++的第一步，打开一个终端将./build/YKS_SDK运行起来
2. 首先打开安装了dds库的python环境，如果没有安装参开dds_int和example/yksddss/文件夹中的readme.md安装环境，包括dds的py库和个人定义的消息库
3. 找到./example/yksddss/yksddspy/z1rc.py
4. 调用其中的squat_control()和read_arm_control()即可，与c++版本使用方法一样


#### 接线说明
1. 接入一个5V电源：将电池线连接至Ethercat板子的电源一端，这一端是靠近STM32芯片的一端，另一端连接至上位机网口。
### 注意，这一步不能接反，不确定请找接过的人！！！
2. 修改电机的can id，然后接到板子上，CAN1通道为电机can id 1，2，3；CAN2通道为电机can id 4，5，6；