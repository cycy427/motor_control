### 使用简要说明，先阅读这个，有需要再阅读下面完整教程

C++使用方法：

0. 安装依赖库：
`sudo apt install libtinfo-dev libreadline-dev libboost-all-dev libncurses5-dev libncursesw5-dev net-tools`

1. 将build文件夹删除，重新创建工程后编译
    ```shell
       mkdir build
       cd build
       cmake .. //这一步如果提示错误,请删除build文件夹,重新创建
       make
       sudo ./YKS_SDK
     cmake .. 这一步如果提示错误,请删除build文件夹,重新创建
     如果还不行，请先解压example中的dds_int，然后按照其中的readme.md安装环境，再重新编译

2. 检查app/motor_control.c文件中的电机型号和映射设置是否正确，如有需要，修改电机型号或接线映射。
   `Z1_MOTOR_ID_Type` 是电机型号顺序，`g_motor_map` / `g_slaves` 是全局电机到 EtherCAT 从站、PDO 槽位、CAN ID 的映射。
3. 检查app/transmit.cpp文件中的Z1_MOTOR_POS_MAX到Z1_MOTOR_TOR_MIN是否正确，这里是限制位置等量（速度和力矩有待实验证明）
4. 检查Z1_legs.h中LegDirectionMotor_数组的电机转向是否正确，如有需要，修改
5. 在main.cpp中，检查第一行CYCLONEDDS_URI的路径是否正确，如有需要，修改
6. 在main.cpp中，检查CAT_Init("enp3s0")中的网口名称是否正确，如有需要，修改
7. 如果使用python控制，检查上述内容后在中终端中打开就行了，换一个终端使用python对齐修改

#### python控制电机运动：

1. 完成c++的第一步，打开一个终端将./build/YKS_SDK运行起来
2. 首先打开安装了dds库的python环境，如果没有安装参开dds_int中的readme.md安装环境和example/yksddss/文件夹中的readme.md安装消息库
3. 例程为./example/yksddss/yksddspy/z1rc.py，认真阅读后可以只打开ethercat板子但是不打开电机进行测试，看./YKS_SDK
   页面是否有收到消息，通信成功后参考这个例程写自己收发程序或者调用这个例程的函数皆可
   比如：No module named 'nubotddsmsg' 进入example/yksddss/文件夹中的readme.md安装消息库

4. 当前主程序使用 leg / arm / body 三路 DDS 分组接口：leg 12 个、body 3 个、arm 14 个。发送程序中的 `index` 使用电机全局 ID，和 `./YKS_SDK` 终端显示以及 `robot_layout.c` 中的主动关节 ID 一致；只控制双足时，仅发送 leg 分组或只更新 `kLegMotorIds` 对应的全局 ID。
5. 当前机器人主动关节不是连续 0-28 编号，而是稀疏全局 ID：双腿见 `kLegMotorIds`，躯干见 `kBodyMotorIds`，双臂见 `kArmMotorIds`。底层 48 个槽位中未列入这些数组的 ID 作为预留或未启用槽位处理。

#### imu使用说明：
1. example/dds_imu/imu_pub.py为imu发布节点，开启一个新终端，进入相关环境运行即可
2. example/dds_imu/imu_sub.py为imu订阅节点，开启一个新终端，进入相关环境运行即可，可以调用其中的函数自己使用

#### 遥控器使用说明：
1. example/logitech/logic_pub.py为logic发布节点，开启一个新终端，进入相关环境运行即可
2. example/logitech/logic_sub.py为logic订阅节点，开启一个新终端，进入相关环境运行即可，可以调用其中的函数自己使用

#### SBUS接收机使用说明：
1. SBUS 默认不启动；当前主程序中的 SBUS 接收机和 DDS 发布节点保持关闭，运行 `sudo ./YKS_SDK` 不会主动打开串口。
2. 如需使用 SBUS，需要在 `main.cpp` 中重新启用 SBUS publisher、`SBusReceiver` 实例和循环发布调用，再运行 `example/yksddss/yksddspy/sbus_sub.py` 订阅数据。

#### 接线说明

1. 接入一个5V电源：将电池线连接至Ethercat板子的电源一端，这一端是靠近STM32芯片的一端，另一端连接至上位机网口。

### 注意，这一步不能接反，不确定请找接过的人！！！

2. 修改电机的 CAN ID，然后接到板子上。默认映射为每个从站 24 个有效槽：槽位 1-8 接 CANFD1 / CAN ID 1-8，槽位 9-16 接 CANFD2 / CAN ID 9-16，槽位 17-24 接 CANFD3 / CAN ID 17-24；第 1 个从站对应底层全局槽位 0-23，第 2 个从站对应底层全局槽位 24-47。CAN ID 是每个从站内的实际电机 ID，不使用跨从站全局编号。当前机器人实际主动关节只使用 `robot_layout.c` 中列出的稀疏全局 ID。

## 调试步骤

规范使用步骤：

- 开启：先ethercat板子上电，然后电机上电，打开sudo ./YKS_SDK，有返回值值之后再用DDS下发指令。
- 关闭：
- （1）先杀死终端中的sudo ./YKS_SDK,再拍急停，此时可以不关闭电池电源（关闭ethercat板子供电），重新打开电机急停开关，电机上电，打开sudo
  ./YKS_SDK，进程重新启动。
- （2）先拍急停，此时电机断电，然后关闭程序，此时不可以直接打开急停和打开sudo ./YKS_SDK，而是要关闭电源开关（关闭ethercat板子供电），重新打开的步骤。
  调试建议：
- 如果有时间的话，先只打开ethercat板子进行调试，通过./YKS_SDK终端观察下发的命令是否正确，确认无误后再关闭ethercat板子电源，重新上述的开启步骤

### 注意事项

1. 底层槽位、从站内 CAN ID 和主站全局 ID 参考app/motor_control.c中的`g_motor_map[].pdo_slot`、`g_motor_map[].can_id`和`g_motor_map[].global_id`；当前主动关节列表参考app/robot_layout.c中的`kLegMotorIds`、`kBodyMotorIds`、`kArmMotorIds`。
2. 请先杀掉./YKS_SDK程序后再拍急停关闭电机，如果先关闭了电机再关程序，请同时将ethercat版也断电，即将电池断电

#### 获得原始数据

1. 调用：z1_leg.cpp里面的EtherCAT_Send_Command（）用户发送数据给ethercat的地方
2. 进入transmit.cpp里面的EtherCAT_Send_Command,根据mode的不同调用不同的解码函数，其中力位混合模式在mode==0的set_ti5_current()
上修改，要拿到原始数据，可以进入这些set_ti5_xxx函数，修改返回值

#### 完整更改消息包

- 改变example/yksddss/nubotidl/nubotddsmsg.idl 的内容，与最外层的nubotddsmsg.idl文件统一

- 使用命令重新生成 nubotddsmsg库
  ```
  idlc -l py nubotddsmsg.idl
  ```
- 重新进入nubotidl文件夹，在该目录下执行 pip install .

#### DDS分组使用简要说明
当前主程序使用 leg / arm / body 三路 DDS 分组收发，不再使用旧的 wholebody `DDS_Z1_5_WB_*` 主程序路径。上层程序需要与底层保持一致：双腿发送 12 个关节、躯干发送 3 个关节、双臂发送 14 个关节，消息中的 `index` 按 `robot_layout.c` 中的全局 ID 填写。


## YKS_SDK 完整说明

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
5. 检查[transmit.h](app/transmit.h)中的从站与槽位容量。当前底层默认保留 2 个 EtherCAT-CANFD 从站、48 个 PDO/全局 ID 槽位容量；当前机器人控制链路启用 29 个主动关节。
6.

SBUS 默认不启动；如果需要使用SBUS接收机，需要先在主程序中启用对应代码，并修改串口的别名，才能找到这个接收机，具体使用教程可以参见 [SBUS转USB串口配置教程](https://www.wolai.com/kUuBkzjtbkCvuwPxWN3Epj)

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

#### Z1Legs闭链踝关节正逆解说明

`Z1Legs` 中的 `Mode::PR` 是默认控制模式。该模式下，上层接口仍然按照机器人逻辑关节发送和读取数据，而不是直接按照脚踝 A/B 两个物理电机控制。

逻辑关节索引如下：

- 左腿脚踝：`4` 表示 `LeftAnklePitch`，`5` 表示 `LeftAnkleRoll`。
- 右腿脚踝：`10` 表示 `RightAnklePitch`，`11` 表示 `RightAnkleRoll`。
- 在底层电机映射中，脚踝仍由 A/B 两个电机耦合驱动：左脚踝 `LeftAnkleB=4`、`LeftAnkleA=5`，右脚踝 `RightAnkleB=10`、`RightAnkleA=11`。

发送命令时，`setMotorCommand()` 会执行逆运动学：上层给定的脚踝 `Pitch/Roll` 期望位置、速度和前馈力矩，会被转换为两个物理脚踝电机 A/B 的命令。位置逆解由 `inverse_kinematics()` 计算，速度通过数值雅可比矩阵换算，前馈力矩按 Pitch/Roll 组合关系分配到 A/B 电机。

读取反馈时，`getMotorData()` 会执行正运动学：先读取 A/B 两个物理电机的实际位置、速度和力矩，再通过 `Pitch_forward_kinematics()` 和 `Roll_forward_kinematics()` 还原为上层看到的脚踝 `Pitch/Roll` 逻辑关节状态。因此在 `sudo ./YKS_SDK` 的全屏状态显示和 DDS whole-body 状态中，索引 `4/5/10/11` 默认表示逻辑脚踝 Pitch/Roll，而不是原始 A/B 电机角度。

正运动学求解内部使用 `forward_kinematics()` 做牛顿迭代，并反复调用 `inverse_kinematics()` 计算残差和数值雅可比。由于机械边界、反馈噪声或初始姿态可能让几何约束中的 `asin()` 输入略微超过 `[-1, 1]`，当前实现会先将该输入钳位到 `[-1, 1]`，并在迭代过程中遇到非有限值时回退，避免单次异常反馈在全屏显示或 DDS 状态中扩散成 `nan`。该钳位是闭链运动学的数值保护，不改变 CAN 返回帧的原始解包结果；如果某个关节持续接近边界或反复触发保护，应优先检查实际脚踝 A/B 电机零位、方向、机械安装和 `LegDirectionMotor_`。

如果需要绕过闭链解算、直接控制脚踝 A/B 两个物理电机，需要将 `Z1Legs` 的模式改为 `Mode::AB` 或自行创建不经过 PR 解算的控制类。直接调试电机时还需要重点检查 `Z1Legs.h` 中的 `LegDirectionMotor_`，确保实际电机正方向与 URDF/控制坐标系一致。所有位置单位为 rad，速度单位为 rad/s，力矩单位为 N.m。

在[motor_control.c](app/motor_control.c)和[motor_control.h](app/motor_control.h)
包含了有关Ethercat板子所接电机型号的设置，以及对应的参数设置，需要使用者提前注意设置好
在[transmit.cpp](app/transmit.cpp)中，包含所接Ti5电机和YKS电机数量的设置，以及最大从站数量的设置，需要使用者根据自己的电机数量进行修改。当前主动关节拓扑以[robot_layout.h](app/robot_layout.h)和[robot_layout.c](app/robot_layout.c)为准：双腿 12 个、躯干 3 个、双臂 14 个，共 29 个主动关节。

当前底层默认适配新的 EtherCAT-CANFD 从站：每个从站预留 40 个 PDO 帧槽，其中前 24 个槽有效；槽位 1-8 对应 CANFD1，9-16 对应 CANFD2，17-24 对应 CANFD3。
底层容量默认使用 2 个 EtherCAT-CANFD 从站，两个从站都启用 24 个有效槽位：底层全局槽位 0-23 接在第 1 个从站槽位 1-24，底层全局槽位 24-47 接在第 2 个从站槽位 1-24。
全局 ID 仅用于主站跨从站区分、数组索引和 DDS 关节映射；实际下发到 CANFD 总线的 CAN ID 按每个从站内部重复编号：槽位 1-24 对应 CAN ID 1-24。硬件侧驱动器 ID 应按所在从站内的本地 CAN ID 配置，而不是按文档全局 ID 配置。当前机器人控制链路只使用其中的稀疏主动关节 ID 集合，具体列表见 `kLegMotorIds`、`kBodyMotorIds`、`kArmMotorIds`。

安装xone手柄驱动：（获取力反馈，并非必须）需要同时安装xone和xpadneo才能用，就听神奇的
https://gitcode.com/gh_mirrors/xo/xone

更多教程参见app文件夹下的[YKS官方教程](app/README.md#SOEM主站)
