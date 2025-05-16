//
// Created by nubot on 25-5-16.
//
// DDSRelate.cpp
#include "DDSRelate.h"


DDSRelate::DDSRelate() {
    // 初始化逻辑（可选）
}

void DDSRelate::DDS_Get_Leg_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    for (int i = 0; i < motor_num; i++) {
        motor_cmds[i].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i].mode = cmds.cmds()[i].mode();
        motor_cmds[i].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i].kd_ = cmds.cmds()[i].kd();
    }
}

void DDSRelate::DDS_Get_Z1_5_WB_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    for (int i = 0; i < motor_num; i++) {
        motor_cmds[i].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i].mode = cmds.cmds()[i].mode();
        motor_cmds[i].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i].kd_ = cmds.cmds()[i].kd();
    }
}

void DDSRelate::DDS_Get_Arm_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    for (int i = 0; i < motor_num; i++) {
        motor_cmds[i + 12].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i + 12].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i + 12].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i + 12].mode = cmds.cmds()[i].mode();
        motor_cmds[i + 12].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i + 12].kd_ = cmds.cmds()[i].kd();
    }
}

void DDSRelate::DDS_Get_Body_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    for (int i = 0; i < motor_num; i++) {
        motor_cmds[i + 24].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i + 24].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i + 24].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i + 24].mode = cmds.cmds()[i].mode();
        motor_cmds[i + 24].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i + 24].kd_ = cmds.cmds()[i].kd();
    }
}

void DDSRelate::DDS_Pub_Arm_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                      const YKSMotorData *motor_data_) {
    int motor_num = ARM_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        auto &state = states.states()[i];
        state.mode(motor_data_[i + 12].mode);
        state.index(i + 12);
        state.pos(motor_data_[i + 12].pos_);
        state.vel(motor_data_[i + 12].vel_);
        state.cur(motor_data_[i + 12].tau_);
        state.tau(motor_data_[i + 12].tau_);
        state.tau_raw(motor_data_[i + 12].tau_);
        state.error(motor_data_[i + 12].error_);
        state.tem(motor_data_[i + 12].temperature_);
        state.mos_tem(motor_data_[i + 12].mos_temperature_);
    }
    writer.write(states);
}

void DDSRelate::DDS_Pub_Leg_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                      const YKSMotorData *motor_data_) {
    int motor_num = LEG_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        auto &state = states.states()[i];
        state.mode(motor_data_[i].mode);
        state.index(i);
        state.pos(motor_data_[i].pos_);
        state.vel(motor_data_[i].vel_);
        state.cur(motor_data_[i].tau_);
        state.tau(motor_data_[i].tau_);
        state.tau_raw(motor_data_[i].tau_);
        state.error(motor_data_[i].error_);
        state.tem(motor_data_[i].temperature_);
        state.mos_tem(motor_data_[i].mos_temperature_);
    }
    writer.write(states);
}

void DDSRelate::DDS_Pub_Z1_5_WB_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                          const YKSMotorData *motor_data_) {
    int motor_num = TOTAL_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        auto &state = states.states()[i];
        state.mode(motor_data_[i].mode);
        state.index(i);
        state.pos(motor_data_[i].pos_);
        state.vel(motor_data_[i].vel_);
        state.cur(motor_data_[i].tau_);
        state.tau(motor_data_[i].tau_);
        state.tau_raw(motor_data_[i].tau_);
        state.error(motor_data_[i].error_);
        state.tem(motor_data_[i].temperature_);
        state.mos_tem(motor_data_[i].mos_temperature_);
    }
    writer.write(states);
}

void DDSRelate::DDS_Pub_Body_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                       const YKSMotorData *motor_data_) {
    int motor_num = BODY_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        auto &state = states.states()[i];
        state.mode(motor_data_[i + 24].mode);
        state.index(i + 24);
        state.pos(motor_data_[i + 24].pos_);
        state.vel(motor_data_[i + 24].vel_);
        state.cur(motor_data_[i + 24].tau_);
        state.tau(motor_data_[i + 24].tau_);
        state.tau_raw(motor_data_[i + 24].tau_);
        state.error(motor_data_[i + 24].error_);
        state.tem(motor_data_[i + 24].temperature_);
        state.mos_tem(motor_data_[i + 24].mos_temperature_);
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
// void DDSRelate::DDS_Z1_5_WB_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
//     samples = Reader.take();
//     if (samples.length() > 0) {
//         for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
//             const motorcmds &cmds = sample_iter->data();
//             const dds::sub::SampleInfo &info = sample_iter->info();
//             if (info.valid()) {
//                 DDS_Get_Z1_5_WB_Motor_Cmds(TOTAL_MOTOR_NUMBER, cmds, my_motor_data);
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
