# 1. yksddsc

- c++代码，我将Z1划分为上肢和下肢。

- 这个代码的功能：
  
  - 发布上肢状态（ARMSTATETOPIC）和下肢状态（LEGSTATETOPIC）两个话题
  
  - 订阅上肢命令（ARMCMDTOPIC）和下肢命令（LEGCMDTOPIC）两个话题

- 编译后即可运行。我已经在代码中作了注释，之需要在相关的地方把数据传递给电机地层控制代码就可以，把Z1legs类引进来即可。

- 要求1：将原有的遥控器等功能剥离。这个节点只负责电机驱动和电机状态反馈。

- 要求2：查看以下个根目录下的idl文件，我对里面的消息进行了规范。命令部分和现有的部分一致。状态反馈部分：我作了细微调整，1是反馈原始读取的电流 2是新增了由我们标定数据估计的力矩 3是按照厂家的系数估计的力矩

# 2. yksddspy

- python代码，和底层交互的接口。

- 准备工作
  
  - 进入nubotidl文件夹，在该目录下执行 pip install . (目的是安装nubotddsmsg消息包)
  
  ```
  z1 = Z1RemoteClient(ARMCMDTOPIC,ARMSTATETOPIC,'arm')
  # z1 = Z1RemoteClient(LEGCMDTOPIC,LEGSTATETOPIC,'leg')
  for i in range(100):
      z1.motorCmds.level = i
      z1.motorCmds.cmds[0].pos = i
      z1.setCommand()
      print("pub %d"%(i))
  
      st = z1.getStates()
      print("sub %d"%(st.states[0].index))
      time.sleep(1)
  ```

- 可以看到Z1RemoteClient可以被定义成上肢或者下肢控制类。

- 该类有一个成员 .motorCmds, 对应电机的控制指令，在主程序中设置好它后，调用setCommand使命令生效。生效的命令会被发布出去。

- 该类通过getStates获得电机的状态。

# 3. 更改消息

- 改变example/yksddss/nubotidl/nubotddsmsg.idl 的内容，与最外层的nubotddsmsg.idl文件统一

- 使用命令重新生成 nubotddsmsg库
  ```
  idlc -l py nubotddsmsg.idl
  ```
- 重新进入nubotidl文件夹，在该目录下执行 pip install .
