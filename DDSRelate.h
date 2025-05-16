//
// Created by luo on 25-5-16.
//
#ifndef DDSRELATE_H
#define DDSRELATE_H

#include "nubotddsmsg.hpp"
#include "app/transmit.h"
#include <dds/dds.hpp>

using namespace org::eclipse::cyclonedds;
using namespace nubotddsmsg::hr;

class DDSRelate {
public:
    DDSRelate(); // 构造函数（可选）

    // 获取电机指令的函数
    void DDS_Get_Leg_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds);
    void DDS_Get_Z1_5_WB_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds);
    void DDS_Get_Arm_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds);
    void DDS_Get_Body_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds);

    // 发布电机状态的函数
    void DDS_Pub_Arm_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                const YKSMotorData *motor_data_);
    void DDS_Pub_Leg_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                const YKSMotorData *motor_data_);
    void DDS_Pub_Z1_5_WB_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                    const YKSMotorData *motor_data_);
    void DDS_Pub_Body_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                 const YKSMotorData *motor_data_);

    // 订阅回调函数
    // void DDS_Arm_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples);
    // void DDS_Leg_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples);
    // void DDS_Z1_5_WB_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples);
    // void DDS_Body_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples);

private:
    // 可以在此处添加私有成员变量或函数
};


#endif //DDSRELATE_H
