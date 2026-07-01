<!--
 * @Description: 主站使用说明
 * @Version: 3.0
 * @Author: Kx Zhang, BismarckOfficial
 * @Date: 2022-11-12 23:34
 * @LastEditTime: 2022-11-21 22:24
-->
## <a id="SOEM"></a>SOEM主站
### 如何部署 使用root权限运行
1. 安装Readline库，该库用于从终端读取用户输入  
   以Ubuntu为例，可以使用`sudo apt install libreadline-dev`指令安装此库
   cmake 报错：Could NOT find Boost (missing: Boost_INCLUDE_DIR)；
   请安装：Boost 输入`sudo apt-get install libboost-all-dev`
2. 使用`ifconfig`指令确定接入从机的网卡名称  `sudo apt-get install net-tools`
3. 将该网卡名称填入到`main.cpp`中、"在while函数中需添加不少于10MS的延时，否则电机无法正常运行"
4. 检查`transmit.h`中的从站和电机数量。当前默认适配2个 EtherCAT-CANFD 从站和48个机器人电机。
5. 编译工程
    ```shell
   mkdir build
   cd build
   cmake .. //这一步如果提示错误，请删除build文件夹，重新创建
   make
    ```
6. 启动程序
    ```shell
   sudo ./master_stack_test
   ```
   由于soem使用到了原始套接字，该程序必须以root权限运行，也可以使用setcap为本程序单独赋予原始套接字权限，
   可以参考[这篇文章](https://squidarth.com/networking/systems/rc/2018/05/28/using-raw-sockets.html)

### 如何使用
本测试程序具备读取/设置电机ID、设置电机为速度伺服控制/位置伺服控制的功能  
* 在启动程序后，可以通过tab补全程序内部指令，上下键切换历史指令，其他快捷键与bash快捷键相同
* 具体可以使用的指令有（括号中为默认值），参数具体含义可以参考电机使用说明书
  * MotorIdGet \<SlaveId\>
  * MotorIdSet \<SlaveId\> \<MotorId\> \<NewMotorId\>
  * MotorSpeedSet \<SlaveId\> \<PdoSlot\> \<CanId\> \<Speed\>(0) \<Current\>(500) \<AckStatus\>(2)
  * MotorPositionSet \<SlaveId\> \<PdoSlot\> \<CanId\> \<Position\>(0) \<Speed\>(50) \<Current\>(500) \<AckStatus\>(2)

其中 `SlaveId` 为0开始的从站数组下标，当前有效范围为0-1；`PdoSlot` 为每个从站内部的PDO槽位，当前有效范围为1-24；`CanId` 为该从站内实际电机 CAN ID，默认同样为1-24，不使用文档全局 ID。

### 特别注意
当前主站适配新的 EtherCAT-CANFD 从站：每个从站预留40个PDO帧槽，前24个槽有效；槽位1-8对应CANFD1，9-16对应CANFD2，17-24对应CANFD3。
完整机器人默认映射为：电机0-23接在第1个从站槽位1-24，电机24-47接在第2个从站槽位1-24。全局 ID 只用于主站区分、数组索引和上层映射；实际 CAN ID 在每个从站内按槽位重复使用1-24，硬件驱动器ID应按所在从站的本地 CAN ID 配置。

### 二次开发
* 多线程功能
  * 主线程，用于接受用户指令并传递
  * 任务线程，循环向从机发送指令并接受返回数据
  * 错误处理线程，处理EtherCat传输错误

* 各文件作用说明
  * main.cpp 程序入口
  * config.h 定义EtherCAT传输的数据结构
  * transmit.cpp/.h 定义EtherCAT主站初始化、运行、获取数据、发送数据函数
  * motor_control.c/.h 定义电机控制相关指令，配合电机说明手册使用
  * math_ops.c/.h 电机控制中用到的一些数学运算
  * queue.h 任务线程通信接口
  * command.cpp/.h 接受用户指令并向任务线程发送

* 向从机发送指令
  * 向从机发送指令需要使用任务线程通信接口，请包含`queue.h`及`motor_control.h`
  * 调用任务线程通信接口可以参考`command.cpp`中的`sendToQueue`方法
    1. 首先需要生成需要下发的msg，queue接受的变量类型为`std::shared_ptr<EtherCAT_Msg>`，
      可以通过`motor_control.h`中提供的方法生成需要的msg
    2. 然后将生成的msg发送到任务线程，`queue.h`中提供了n条无锁消息队列（n为最大从机数），
      取出目标从机对应的消息队列，将msg推入其中，任务线程将在每次循环自动取出一条消息发送到从机
* 从从机接受消息
  * demo中没有实现从机消息回传到主线程，如果您的应用场景需要使用从机回传数据，请在queue中建立n条新消息队列
    （仅为extern声明，全局变量声明需放置于`transmit.cpp`）
  * 并在`transmit.cpp`的`EtherCAT_Data_Get`方法中将回传消息压入消息队列，对于回传消息的解析可以参考
    `motor_control.cpp`中的`Rv_Message_Print`方法
  * 最后在需要使用回传数据的线程循环从队列中取出消息即可（注意：`queue.h`中目前使用的是环形缓冲区队列，
    仅能单读单写，如有其他需求请更换队列模型）
### 常见问题及解决方案
* 从站板上电后使用ifconfig无法检测到网口号
  使用虚拟机时需要注意虚拟机与主机之间的网络连接方式，尽可能使用双系统或者Linux系统
  请检测网线连接是否正常。
* ethercat板子连上主站时会断
  检查程序中是否有线程干扰；给主站连接程序独立线程进行测试；检测程序运行中是否运行时间过长，超过了从站板最大等待时间
* ethercat板子在连接时经常连接不上
  保持稳定供电，从站板供电需要至少500mA的电流，同时不同主站运行速度不同，可能会有所影响
