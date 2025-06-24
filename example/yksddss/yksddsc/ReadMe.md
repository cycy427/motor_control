---
author: GoldenPhilosophy
date: 2025-03-11
---
### 使用简要说明 这个对应于解耦，解耦和不解耦不能同时使用,请确保底层和上层一致
1. 解耦：底层main.cpp注释掉DDS_Z1_5_WB_SUB(z1_5_wb_Reader, samples_z1_5_wb);和DDS_Pub_Z1_5_WB_Motor_Data(z1_5_wb_States, z1_5_wb_Writer, my_motor_data);
2. 不解耦：底层main.cpp注释掉DDS_Leg_SUB，DDS_Arm_SUB，DDS_Body_SUB，DDS_Pub_Arm_Motor_Data，DDS_Pub_Leg_Motor_Data，DDS_Pub_Body_Motor_Data

#### 类型说明
1. 有两个，一个是上肢，一个是双肩和腰部
2. 代码中setArmYKSSquatControl以下的可以使用

**C++使用方法：**

0. 安装依赖库：
`sudo apt install libtinfo-dev libreadline-dev libboost-all-dev libncurses5-dev libncursesw5-dev net-tools`

1. 将build文件夹删除，重新创建工程后编译
    ```shell
       mkdir build
       cd build
       cmake .. //这一步如果提示错误,请删除build文件夹,重新创建
       make
       sudo ./z1_body

2. cmake .. 这一步如果提示错误,请删除build文件夹,重新创建 
如果还不行，请先解压example中的dds_int，然后按照其中的readme.md安装环境，再重新编译

3. 发送命令：my_motor_data中,修改my_motor_data中的数据，然后发送，如：        
   ```
   my_motor_data[0].mode =  0;
   my_motor_data[0].pos_des_ = 1;
   my_motor_data[0].vel_des_ = 0;
   my_motor_data[0].ff_ = 0;
   my_motor_data[0].kp_ = 100;
   my_motor_data[0].kd_ = 10;

4. 读取数据：my_motor_data中剩余的变量，不要修改那些变量，在同一个全局变量中
