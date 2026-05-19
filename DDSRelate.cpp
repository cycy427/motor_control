//
// Created by nubot on 25-5-16.
//
// DDSRelate.cpp
#include "DDSRelate.h"
#include <algorithm>


DDSRelate::DDSRelate() {
    // 初始化逻辑（可选）
}

void DDSRelate::DDS_Get_Leg_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        const int gid = kLegMotorIds[i];
        motor_cmds[gid].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[gid].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[gid].ff_ = cmds.cmds()[i].tau();
        motor_cmds[gid].mode = cmds.cmds()[i].mode();
        motor_cmds[gid].kp_ = cmds.cmds()[i].kp();
        motor_cmds[gid].kd_ = cmds.cmds()[i].kd();
    }
}

void DDSRelate::DDS_Get_Arm_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        const int gid = kArmMotorIds[i];
        motor_cmds[gid].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[gid].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[gid].ff_ = cmds.cmds()[i].tau();
        motor_cmds[gid].mode = cmds.cmds()[i].mode();
        motor_cmds[gid].kp_ = cmds.cmds()[i].kp();
        motor_cmds[gid].kd_ = cmds.cmds()[i].kd();
    }
}

void DDSRelate::DDS_Get_Body_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        const int gid = kBodyMotorIds[i];
        motor_cmds[gid].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[gid].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[gid].ff_ = cmds.cmds()[i].tau();
        motor_cmds[gid].mode = cmds.cmds()[i].mode();
        motor_cmds[gid].kp_ = cmds.cmds()[i].kp();
        motor_cmds[gid].kd_ = cmds.cmds()[i].kd();
    }
}

void DDSRelate::DDS_Pub_Arm_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                      const YKSMotorData *motor_data_) {
    int motor_num = ARM_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        const int gid = kArmMotorIds[i];
        auto &state = states.states()[i];
        state.mode(motor_data_[gid].mode);
        state.index(gid);
        state.pos(motor_data_[gid].pos_);
        state.vel(motor_data_[gid].vel_);
        state.cur(motor_data_[gid].tau_);
        state.tau(motor_data_[gid].tau_);
        state.tau_raw(motor_data_[gid].tau_);
        state.error(motor_data_[gid].error_);
        state.tem(motor_data_[gid].temperature_);
        state.mos_tem(motor_data_[gid].mos_temperature_);
    }
    writer.write(states);
}

void DDSRelate::DDS_Pub_Leg_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                      const YKSMotorData *motor_data_) {
    int motor_num = LEG_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        const int gid = kLegMotorIds[i];
        auto &state = states.states()[i];
        state.mode(motor_data_[gid].mode);
        state.index(gid);
        state.pos(motor_data_[gid].pos_);
        state.vel(motor_data_[gid].vel_);
        state.cur(motor_data_[gid].tau_);
        state.tau(motor_data_[gid].tau_);
        state.tau_raw(motor_data_[gid].tau_);
        state.error(motor_data_[gid].error_);
        state.tem(motor_data_[gid].temperature_);
        state.mos_tem(motor_data_[gid].mos_temperature_);
    }
    writer.write(states);
}

void DDSRelate::DDS_Pub_Body_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                       const YKSMotorData *motor_data_) {
    int motor_num = BODY_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        const int gid = kBodyMotorIds[i];
        auto &state = states.states()[i];
        state.mode(motor_data_[gid].mode);
        state.index(gid);
        state.pos(motor_data_[gid].pos_);
        state.vel(motor_data_[gid].vel_);
        state.cur(motor_data_[gid].tau_);
        state.tau(motor_data_[gid].tau_);
        state.tau_raw(motor_data_[gid].tau_);
        state.error(motor_data_[gid].error_);
        state.tem(motor_data_[gid].temperature_);
        state.mos_tem(motor_data_[gid].mos_temperature_);
    }
    writer.write(states);
}

// void DDSRelate::DDS_Arm_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
//     samples = Reader.take();
//     if (samples.length() > 0) {
//         for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
//             const motorcmds &cmds = sample_iter->data();
//             const dds::sub::SampleInfo &info = sample_iter->info();
//             if (info.valid()) {
//                 DDS_Get_Arm_Motor_Cmds(ARM_MOTOR_NUMBER, cmds, my_motor_data);
//             }
//         }
//     }
// }
//
// void DDSRelate::DDS_Leg_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
//     samples = Reader.take();
//     if (samples.length() > 0) {
//         for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
//             const motorcmds &cmds = sample_iter->data();
//             const dds::sub::SampleInfo &info = sample_iter->info();
//             if (info.valid()) {
//                 DDS_Get_Leg_Motor_Cmds(LEG_MOTOR_NUMBER, cmds, my_motor_data);
//             }
//         }
//     }
// }
//
// void DDSRelate::DDS_Body_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
//     samples = Reader.take();
//     if (samples.length() > 0) {
//         for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
//             const motorcmds &cmds = sample_iter->data();
//             const dds::sub::SampleInfo &info = sample_iter->info();
//             if (info.valid()) {
//                 DDS_Get_Body_Motor_Cmds(BODY_MOTOR_NUMBER, cmds, my_motor_data);
//             }
//         }
//     }
// }
