#include "SBusReceiver.h"
#include "JoyStickHandler.h"
#include "Z1Legs.h"
#include "MotorDataLogger.h"
#include "BmsHandler.h"
#include "SocketReceiver.h"
#include "SocketSender.h"
#include "dds/dds.hpp"
#include "nubotddsmsg.hpp"
#include <mutex>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <csignal> // 信号头文件
#include <unistd.h>   // write(), STDOUT_FILENO
#include <cstring>   // strlen()
#define DDS  //如果想要使用Socket通信，那么就注释，如果想使用DDS通信，那么请取消注释

//这里以后考虑参数传递或者文件配置主题名称
#define ARMCMDTOPIC "/nubot/z1/armmotorcmds"
#define LEGCMDTOPIC "/nubot/z1/legmotorcmds"

#define ARMSTATETOPIC "/nubot/z1/armmotorstates"
#define LEGSTATETOPIC "/nubot/z1/legmotorstates"

#define BODYCMDTOPIC "/nubot/z1/bodymotorcmds"
#define BODYSTATETOPIC "/nubot/z1/bodymotorstates"

#define WHOLEBODYCMDTOPIC  "/nubot/z1/wholebodymotorcmds"
#define WHOLEBODYSTATETOPIC  "/nubot/z1/wholebodymotorstates"

#define SBUSSTATETOPIC  "/nubot/z1/wholebodymotorstates"

#define IMUPUBDATATOPIC  "/nubot/z1/imupubdata"
#define HCMDPUBDATATOPIC  "/nubot/z1/hcmdpubdata"
#define LOGICPUBDATATOPIC  "/nubot/z1/logicpubdata"
#define BMSPUBDATATOPIC  "/nubot/z1/bmspubdata"

using namespace org::eclipse::cyclonedds;
using namespace nubotddsmsg::hr;
using namespace nubotddsmsg::sensor;
using namespace nubotddsmsg::logic;
using namespace nubotddsmsg::hcmd;
using namespace nubotddsmsg::sbus;
using namespace nubotddsmsg::bms;

// 用于同步 imu 状态的 flag 和线程唤醒
std::mutex imu_mutex;
std::condition_variable imu_cv;
// 用于同步 hcmd 状态的 flag 和线程唤醒
std::mutex hcmd_mutex;
std::condition_variable hcmd_cv;
// 用于同步 logic 状态的 flag 和线程唤醒
std::mutex logic_mutex;
std::condition_variable logic_cv;
YKSMotorData my_motor_data[Z1_NUM_MOTOR];
bool is_imu_run = false;
bool is_hcmd_run = false;
bool is_logic_run = false;
bool is_bms_run = false;

std::atomic<bool> stop_thread(false);
volatile sig_atomic_t stop_flag = false; // 使用不带 std:: 的 sig_atomic_t
void signalHandler(int signal) {
    if (signal == SIGINT) {
        stop_flag = true;
        const char *msg = "\nSIGINT received, preparing to exit...\n";
        write(STDOUT_FILENO, msg, std::strlen(msg));
    }
}

///非常简单的测试函数，用于测试下肢的运动控制
void squat_control(const float pos) {
    // my_motor_data[Z1JointIndex::LeftHipYaw].pos_des_ = pos * 0.2; //左右转动
    my_motor_data[Z1JointIndex::LeftHipYaw].pos_des_ = 0; //左右转动
    my_motor_data[Z1JointIndex::LeftHipPitch].pos_des_ = -pos * 0.25; //前后运动
    // my_motor_data[Z1JointIndex::LeftHipRoll].pos_des_ = pos * 0.2;//外摆
    my_motor_data[Z1JointIndex::LeftHipRoll].pos_des_ = 0;
    my_motor_data[Z1JointIndex::LeftKnee].pos_des_ = pos * 0.7;
    my_motor_data[Z1JointIndex::LeftAnkleA].pos_des_ = -pos * 0.1;
    my_motor_data[Z1JointIndex::LeftAnkleB].pos_des_ = pos * 0.1;

    // my_motor_data[Z1JointIndex::RightHipYaw].pos_des_ = pos * 0.2; //左右转动
    my_motor_data[Z1JointIndex::RightHipYaw].pos_des_ = 0; //左右转动
    my_motor_data[Z1JointIndex::RightHipPitch].pos_des_ = pos * 0.25; //ok
    // my_motor_data[Z1JointIndex::RightHipRoll].pos_des_ = -pos * 0.2 - 0.1; //8
    my_motor_data[Z1JointIndex::RightHipRoll].pos_des_ = 0;
    my_motor_data[Z1JointIndex::RightKnee].pos_des_ = -pos * 0.7; //ok
    my_motor_data[Z1JointIndex::RightAnkleA].pos_des_ = pos * 0.1;
    my_motor_data[Z1JointIndex::RightAnkleB].pos_des_ = -pos * 0.1;

    // my_motor_data[Z1JointIndex::WaistYaw].pos_des_ = pos;
}
//置零函数
void zero_out() {
    for (int i = 0; i < Z1_NUM_MOTOR; i++) {
        my_motor_data[i].pos_des_ = 0;
        my_motor_data[i].vel_des_ = 0;
        my_motor_data[i].ff_ = 0;
        my_motor_data[i].mode = 0;
        my_motor_data[i].kp_ = 0;
        my_motor_data[i].kd_ = 0;
    }
}

void DDS_Get_Leg_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        motor_cmds[i].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i].mode = cmds.cmds()[i].mode();
        motor_cmds[i].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i].kd_ = cmds.cmds()[i].kd();
    }
}

void DDS_Get_Z1_5_WB_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        motor_cmds[i].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i].mode = cmds.cmds()[i].mode();
        motor_cmds[i].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i].kd_ = cmds.cmds()[i].kd();
    }
}

void DDS_Get_Arm_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        motor_cmds[i + 12].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i + 12].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i + 12].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i + 12].mode = cmds.cmds()[i].mode();
        motor_cmds[i + 12].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i + 12].kd_ = cmds.cmds()[i].kd();
    }
}

void DDS_Get_Body_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        motor_cmds[i + 24].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i + 24].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i + 24].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i + 24].mode = cmds.cmds()[i].mode();
        motor_cmds[i + 24].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i + 24].kd_ = cmds.cmds()[i].kd();
    }
}

// void DDS_Get_IMU_States(const int motor_num, const imudata &cmds, YKSMotorData *motor_cmds) {
//     // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
//     for (int i = 0; i < motor_num; i++) {
//         motor_cmds[i].pos_des_ = cmds.cmds()[i].pos();
//         motor_cmds[i].vel_des_ = cmds.cmds()[i].vel();
//         motor_cmds[i].ff_ = cmds.cmds()[i].tau();
//         motor_cmds[i].mode = cmds.cmds()[i].mode();
//         motor_cmds[i].kp_ = cmds.cmds()[i].kp();
//         motor_cmds[i].kd_ = cmds.cmds()[i].kd();
//     }
// }

void DDS_Pub_Arm_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                            const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
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

void DDS_Pub_Leg_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                            const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
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

void DDS_Pub_Z1_5_WB_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
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

void DDS_Pub_Sbus_Data(sbusdata &states, dds::pub::DataWriter<sbusdata> &writer,
                       const SBusData *sbus_data_) {
    // 发布SBUS数据
    states.lost_frame(sbus_data_->lost_frame);
    states.failsafe(sbus_data_->failsafe);
    std::array<int32_t, 16> ch_values;

    for (size_t i = 0; i < 16; ++i) {
        ch_values[i] = sbus_data_->ch[i];
    }
    states.ch(ch_values);

    writer.write(states);
}

void DDS_Pub_Body_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                             const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
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

void DDS_Arm_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const motorcmds &cmds = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                DDS_Get_Arm_Motor_Cmds(ARM_MOTOR_NUMBER, cmds, my_motor_data);
            }
        }
    }
}

void DDS_Leg_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const motorcmds &cmds = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                DDS_Get_Leg_Motor_Cmds(LEG_MOTOR_NUMBER, cmds, my_motor_data);
            }
        }
    }
}

void DDS_Z1_5_WB_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const motorcmds &cmds = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                DDS_Get_Z1_5_WB_Motor_Cmds(TOTAL_MOTOR_NUMBER, cmds, my_motor_data);
            }
        }
    }
}


void DDS_Body_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const motorcmds &cmds = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                DDS_Get_Body_Motor_Cmds(BODY_MOTOR_NUMBER, cmds, my_motor_data);
            }
        }
    }
}

void DDS_IMU_SUB(dds::sub::DataReader<imudata> &Reader, dds::sub::LoanedSamples<imudata> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const imudata &states = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                is_imu_run = true;
                // DDS_Get_IMU_States(TOTAL_MOTOR_NUMBER, states, my_motor_data);
            }
        }
    } else {
        // is_imu_run = false;
        // std::cout << "no imu data received" << std::endl;
    }
}

void DDS_HCMD_SUB(dds::sub::DataReader<hcmddata> &Reader, dds::sub::LoanedSamples<hcmddata> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const hcmddata &states = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                is_hcmd_run = true;

                // DDS_Get_IMU_States(TOTAL_MOTOR_NUMBER, states, my_motor_data);
            }
        }
    } else {
        // is_hcmd_run = false;
        // std::cout << "no hcmd data received" << std::endl;
    }
}

void DDS_LOGIC_SUB(dds::sub::DataReader<logicdata> &Reader, dds::sub::LoanedSamples<logicdata> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const logicdata &states = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                is_logic_run = true;

                // DDS_Get_IMU_States(TOTAL_MOTOR_NUMBER, states, my_motor_data);
            }
        }
    } else {
        // is_logic_run = false;
        // std::cout << "no logic data received" << std::endl;
    }
}

void DDS_BMS_SUB(dds::sub::DataReader<bmsdata_short> &Reader, dds::sub::LoanedSamples<bmsdata_short> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const bmsdata_short &states = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                is_bms_run = true;

                // DDS_Get_IMU_States(TOTAL_MOTOR_NUMBER, states, my_motor_data);
            }
        }
    } else {
        // is_logic_run = false;
        // std::cout << "no logic data received" << std::endl;
    }
}

int main() {
    // 注册信号处理函数
    std::signal(SIGINT, signalHandler);
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // setenv("CYCLONEDDS_URI", "file:///home/amov/humanoid_proj/z1_rl/dds_helper/dds_config/NetworkInterface.xml", 1); // 覆盖当前进程的环境变量
    // const char *uri = getenv("CYCLONEDDS_URI"); // 验证
    // std::cout << "Active URI: " << (uri ? uri : "NULL") << std::endl;
    // DDS相关处理 ////////////////////////////////////////////////////////////////////////////////////
    dds::domain::DomainParticipant participant(0);
    if (participant == dds::core::null) {
        std::cerr << "Failed to create participant!" << std::endl;
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////
    // 订阅 //////////////////////////////////////////////////////////////////////////////////////////
    // 下肢订阅 ========================================================================================
    //定义下肢订阅者话题
    dds::topic::Topic<motorcmds> legtopicsub(participant, LEGCMDTOPIC);
    //定义下肢订阅者话题Qos
    dds::sub::Subscriber legSubscriber(participant);
    dds::sub::qos::DataReaderQos legReadQos = legSubscriber.default_datareader_qos();
    legReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义下肢Reader
    dds::sub::DataReader<motorcmds> legReader(legSubscriber, legtopicsub, legReadQos);
    std::cout << "=== [Leg subscriber] get ready! " << std::endl;

    // 上肢订阅 ======================================================================================
    //定义上肢订阅者话题
    dds::topic::Topic<motorcmds> armtopicsub(participant, ARMCMDTOPIC);
    //定义下肢订阅者话题Qos
    dds::sub::Subscriber armSubscriber(participant);
    dds::sub::qos::DataReaderQos armReadQos = armSubscriber.default_datareader_qos();
    armReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义上肢Reader
    dds::sub::DataReader<motorcmds> armReader(armSubscriber, armtopicsub, armReadQos);
    std::cout << "=== [Arm subscriber] get ready! " << std::endl;

    // 躯干订阅 ========================================================================================
    //定义躯干订阅者话题
    dds::topic::Topic<motorcmds> bodytopicsub(participant, BODYCMDTOPIC);
    //定义躯干订阅者话题Qos
    dds::sub::Subscriber bodySubscriber(participant);
    dds::sub::qos::DataReaderQos bodyReadQos = bodySubscriber.default_datareader_qos();
    bodyReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义躯干Reader
    dds::sub::DataReader<motorcmds> bodyReader(bodySubscriber, bodytopicsub, bodyReadQos);
    std::cout << "=== [body subscriber] get ready! " << std::endl;

    // z1_5_wb订阅 ========================================================================================
    //定义z1_5_wb订阅者话题
    dds::topic::Topic<motorcmds> z1_5_wb_topicsub(participant, WHOLEBODYCMDTOPIC);
    //定义z1_5_wb订阅者话题Qos
    dds::sub::Subscriber z1_5_wb_Subscriber(participant);
    dds::sub::qos::DataReaderQos z1_5_wb_ReadQos = z1_5_wb_Subscriber.default_datareader_qos();
    z1_5_wb_ReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义z1_5_wbReader
    dds::sub::DataReader<motorcmds> z1_5_wb_Reader(z1_5_wb_Subscriber, z1_5_wb_topicsub, z1_5_wb_ReadQos);
    std::cout << "=== [z1_5_wb subscriber] get ready! " << std::endl;

    // imu订阅 ========================================================================================
    //定义imu订阅者话题
    dds::topic::Topic<imudata> imu_topicsub(participant, IMUPUBDATATOPIC);
    //定义imu订阅者话题Qos
    dds::sub::Subscriber imu_Subscriber(participant);
    dds::sub::qos::DataReaderQos imu_ReadQos = imu_Subscriber.default_datareader_qos();
    imu_ReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义imu_Reader
    dds::sub::DataReader<imudata> imu_Reader(imu_Subscriber, imu_topicsub, imu_ReadQos);
    std::cout << "=== [imu subscriber] get ready! " << std::endl;

    // hcmd订阅 ========================================================================================
    //定义z1_5_wb订阅者话题
    dds::topic::Topic<hcmddata> hcmd_topicsub(participant, HCMDPUBDATATOPIC);
    //定义z1_5_wb订阅者话题Qos
    dds::sub::Subscriber hcmd_Subscriber(participant);
    dds::sub::qos::DataReaderQos hcmd_ReadQos = hcmd_Subscriber.default_datareader_qos();
    hcmd_ReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义hcmd_Reader
    dds::sub::DataReader<hcmddata> hcmd_Reader(hcmd_Subscriber, hcmd_topicsub, hcmd_ReadQos);
    std::cout << "=== [hcmd subscriber] get ready! " << std::endl;

    // logic订阅 ========================================================================================
    //定义logic订阅者话题
    dds::topic::Topic<logicdata> logic_topicsub(participant, LOGICPUBDATATOPIC);
    //定义logic订阅者话题Qos
    dds::sub::Subscriber logic_Subscriber(participant);
    dds::sub::qos::DataReaderQos logic_ReadQos = logic_Subscriber.default_datareader_qos();
    logic_ReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义logic_Reader
    dds::sub::DataReader<logicdata> logic_Reader(logic_Subscriber, logic_topicsub, logic_ReadQos);
    std::cout << "=== [logic subscriber] get ready! " << std::endl;

    // bms订阅 ========================================================================================
    //定义bms订阅者话题
    dds::topic::Topic<bmsdata_short> bms_topicsub(participant, BMSPUBDATATOPIC);
    //定义bms订阅者话题Qos
    dds::sub::Subscriber bms_Subscriber(participant);
    dds::sub::qos::DataReaderQos bms_ReadQos = bms_Subscriber.default_datareader_qos();
    bms_ReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义bms_Reader
    dds::sub::DataReader<bmsdata_short> bms_Reader(bms_Subscriber, bms_topicsub, bms_ReadQos);
    std::cout << "=== [bms subscriber] get ready! " << std::endl;
    // //////////////////////////////////////////////////////////////////////////////////////////////////
    // // 发布 ///////////////////////////////////////////////////////////////////////////////////////////
    // // 下肢发布 =======================================================================================
    // // 定义下肢发布者话题
    dds::topic::qos::TopicQos legtopicpubQos;
    legtopicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息
    dds::topic::Topic<motorstates> legtopicpub(participant, LEGSTATETOPIC);
    // 创建 Publisher 和 DataWriter
    dds::pub::Publisher legPublisher(participant);
    dds::pub::qos::DataWriterQos legwriterQos(legtopicpubQos); // datawriter的qos应当继承自topic的qos
    dds::pub::DataWriter<motorstates> legWriter(legPublisher, legtopicpub, legwriterQos);
    std::cout << "=== [Leg publisher] get ready! " << std::endl;

    // // 上肢发布 =======================================================================================
    // // 定义上肢发布者话题
    dds::topic::qos::TopicQos armtopicpubQos;
    armtopicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息
    dds::topic::Topic<motorstates> armtopicpub(participant, ARMSTATETOPIC);
    // 创建 Publisher 和 DataWriter
    dds::pub::Publisher armPublisher(participant);
    dds::pub::qos::DataWriterQos armwriterQos(armtopicpubQos); // datawriter的qos应当继承自topic的qos
    dds::pub::DataWriter<motorstates> armWriter(armPublisher, armtopicpub, armwriterQos);
    std::cout << "=== [arm publisher] get ready! " << std::endl;

    // // 躯干发布 =======================================================================================
    // // 定义躯干发布者话题
    dds::topic::qos::TopicQos bodytopicpubQos;
    bodytopicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息
    dds::topic::Topic<motorstates> bodytopicpub(participant, BODYSTATETOPIC);
    // 创建 Publisher 和 DataWriter
    dds::pub::Publisher bodyPublisher(participant);
    dds::pub::qos::DataWriterQos bodywriterQos(bodytopicpubQos); // datawriter的qos应当继承自topic的qos
    dds::pub::DataWriter<motorstates> bodyWriter(bodyPublisher, bodytopicpub, bodywriterQos);
    std::cout << "=== [body publisher] get ready! " << std::endl;

    // z1_5_wb发布 =======================================================================================
    // 定义z1_5_wb发布者话题
    dds::topic::qos::TopicQos z1_5_wb_topicpubQos;
    z1_5_wb_topicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息
    dds::topic::Topic<motorstates> z1_5_wb_topicpub(participant, WHOLEBODYSTATETOPIC);
    // 创建 Publisher 和 DataWriter
    dds::pub::Publisher z1_5_wb_Publisher(participant);
    dds::pub::qos::DataWriterQos z1_5_wb_writerQos(z1_5_wb_topicpubQos); // datawriter的qos应当继承自topic的qos
    dds::pub::DataWriter<motorstates> z1_5_wb_Writer(z1_5_wb_Publisher, z1_5_wb_topicpub, z1_5_wb_writerQos);
    std::cout << "=== [z1_5_wb publisher] get ready! " << std::endl;

    // // // SBUS发布 =======================================================================================
    // // // 定义SBUS发布者话题
    // dds::topic::qos::TopicQos sbus_topicpubQos;
    // sbus_topicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
    //         << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
    //         << dds::core::policy::History::KeepLast(5); //保留近5条消息
    // dds::topic::Topic<sbusdata> sbus_topicpub(participant, SBUSSTATETOPIC);
    // // 创建 Publisher 和 DataWriter
    // dds::pub::Publisher sbus_Publisher(participant);
    // dds::pub::qos::DataWriterQos sbus_writerQos(sbus_topicpubQos); // datawriter的qos应当继承自topic的qos
    // dds::pub::DataWriter<sbusdata> sbus_Writer(sbus_Publisher, sbus_topicpub, sbus_writerQos);
    // std::cout << "=== [SBUS publisher] get ready! " << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////

    if (bool Ethernet_Status = CAT_Init("eno1"); !Ethernet_Status) { exit(1); } //如果初始化失败，则直接退出程序
    // auto joystick_device = "/dev/input/js0";
    // auto battery = "/dev/ttyUSB1";
    // const auto joystick_handler = std::make_shared<JoyStickHandler>(joystick_device);
    // const auto battery_handler = std::make_shared<BmsHandler>(battery);
    // const SBusReceiver sbus_receiver("/dev/ttyACM0");

    Z1Legs z1_legs;
    // z1_legs.setJoyStickHandler(joystick_handler);
    // MotorDataLogger motor_data_logger; //创建电机数据记录对象

    // z1_legs.setBatteryHandler(battery_handler);

    // SocketReceiver receiver(YKS_PORT);
    // SocketSender sender("127.0.0.1",USR_PORT);
    // receiver.startListening();
    // sender.sendDataPeriodically();

    motorstates armStates;
    armStates.level(1); // 设置为上肢
    armStates.states().resize(ARM_MOTOR_NUMBER);

    motorstates legStates;
    legStates.level(0); // 设置为下肢
    legStates.states().resize(LEG_MOTOR_NUMBER);

    motorstates bodyStates;
    bodyStates.level(2); // 设置为躯干
    bodyStates.states().resize(BODY_MOTOR_NUMBER);

    motorstates z1_5_wb_States;
    z1_5_wb_States.level(3); // 设置为z1_5_wb
    z1_5_wb_States.states().resize(TOTAL_MOTOR_NUMBER);

    sbusdata sbus_States;


    dds::sub::LoanedSamples<motorcmds> samples_leg;
    dds::sub::LoanedSamples<motorcmds> samples_arm;
    dds::sub::LoanedSamples<motorcmds> samples_body;
    dds::sub::LoanedSamples<motorcmds> samples_z1_5_wb;
    dds::sub::LoanedSamples<imudata> samples_imu;
    dds::sub::LoanedSamples<hcmddata> samples_hcmd;
    dds::sub::LoanedSamples<logicdata> samples_logic;
    dds::sub::LoanedSamples<bmsdata_short> samples_bms;

    while (!stop_flag) {
        //读取imu订阅的消息 -----------------------------------------------------------------------------
        DDS_IMU_SUB(imu_Reader, samples_imu); //
        z1_legs.getIMUFlag(is_imu_run); // 通知后执行对应操作

        //读取hcmd订阅的消息 -----------------------------------------------------------------------------
        DDS_HCMD_SUB(hcmd_Reader, samples_hcmd); //
        z1_legs.getHcmdFlag(is_hcmd_run);

        //读取logic订阅的消息 -----------------------------------------------------------------------------
        DDS_LOGIC_SUB(logic_Reader, samples_logic); //
        z1_legs.getLogicFlag(is_logic_run);

        //读取bms订阅的消息 -----------------------------------------------------------------------------
        DDS_BMS_SUB(bms_Reader, samples_bms); //
        z1_legs.getBmsFlag(is_bms_run);

        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //读取周期为1ms

        if (true) {
        // if (is_logic_run == true && is_imu_run == true ) {
            break;
        }
        printf("Error!!!!!!!!!! Please connect imu and switch!!!!!!!!!!");
    }


    // double pos_pitch = 0;
    // double pos_roll = 0;
    while (!stop_flag) {

#ifdef DDS
        ///////////////////////////////////////////////////////////////////////////////////////////
        //读取leg订阅的消息 ---------------------------------------------------------------------------
        DDS_Leg_SUB(legReader, samples_leg);
        /////////////////////////////////////////////////////////////////////////////////////////////
        //读取arm订阅的消息 -----------------------------------------------------------------------------
        DDS_Arm_SUB(armReader, samples_arm); //
        //读取body订阅的消息 -----------------------------------------------------------------------------
        DDS_Body_SUB(bodyReader, samples_body); //
        //消息断开连接处理
        // if (is_hcmd_run == true && is_imu_run == true && is_logic_run == true) {
        //     //读取Z1_5_WB订阅的消息 -----------------------------------------------------------------------------
        //     DDS_Z1_5_WB_SUB(z1_5_wb_Reader, samples_z1_5_wb);
        // }
        //读取Z1_5_WB订阅的消息 -----------------------------------------------------------------------------
        DDS_Z1_5_WB_SUB(z1_5_wb_Reader, samples_z1_5_wb);


        /////////////////////////////////////////////////////////////////////////////////////////////
#endif
#ifndef DDS
        //读取Socket通信的消息 -----------------------------------------------------------------------------
        // receiver.getSocketMotorCMD(my_motor_data);
        /////////////////////////////////////////////////////////////////////////////////////////////
#endif
        //电机执行指令 -----------------------------------------------------------------------------
        z1_legs.setMotorKpKd(my_motor_data); //专门设置电机KP、KD值，调用了这个函数之后就会将原来设置在Z1legs类里面的默认KP、KD值覆盖掉
        // printf("[Motor11] %f\n",my_motor_data[1].pos_des_);
        // printf("[Motor12] %f\n",my_motor_data[11].pos_des_);

        z1_legs.setMotorCommand(my_motor_data); //设置电机指令
        ////////////////////////////////////////////////////////////////////////////////////////////
        //获取所有电机的状态
        z1_legs.getMotorData(my_motor_data); //获取电机数据

        // motor_data_logger.print_log(my_motor_data); //保存电机数据
        ///////////////////////////////////////////////////////////////////////////////////////////
        ///将上肢电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        DDS_Pub_Arm_Motor_Data(armStates, armWriter, my_motor_data);
        ///将下肢电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        DDS_Pub_Leg_Motor_Data(legStates, legWriter, my_motor_data);
        ///将躯干电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        DDS_Pub_Body_Motor_Data(bodyStates, bodyWriter, my_motor_data);
        // 将Z1_5_WB电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        DDS_Pub_Z1_5_WB_Motor_Data(z1_5_wb_States, z1_5_wb_Writer, my_motor_data);
        ///通过Socket将电机状态发送给用户端
        // sender.sendSocketMotorData(my_motor_data); //通过Socket反馈电机当前的数据
        // SBusData data = sbus_receiver.getData();
        // SBusReceiver::print_data(data);

        // DDS_Pub_Sbus_Data(sbus_States, sbus_Writer, &data);
        // pos = data.ch[2] / 672.0 * 4;
        // JoystickState state = joystick_handler->getState();
        // JoyStickHandler::print_state(state);
        // squat_control(pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //读取周期为1ms
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); //休息5ms

    zero_out();//数据清零
    z1_legs.setMotorKpKd(my_motor_data); //专门设置电机KP、KD值，调用了这个函数之后就会将原来设置在Z1legs类里面的默认KP、KD值覆盖掉
    z1_legs.setMotorCommand(my_motor_data); //设置电机指令

    std::this_thread::sleep_for(std::chrono::milliseconds(5)); //休息5ms

    stop_thread = true;
    if (runThread.joinable()) {
        runThread.join(); // 确保线程安全退出
        printf("[EtherCAT] Stopping runImpl accomplished.\n");
    }
    pthread_join(*checkThread, nullptr);
    printf("[EtherCAT] Stopping ethercat_check accomplished.\n");

    std::signal(SIGINT, SIG_DFL); // 恢复默认行为（程序立即退出）
    // runThread.join(); //runThread.join(); 的主要功能是确保 main 函数在退出之前等待 runThread 线程完成其任务。
    // 这样做的目的是为了确保程序在退出前所有的后台任务都得到了正确地执行和清理，避免数据丢失或资源泄露。
    return 0;
}
