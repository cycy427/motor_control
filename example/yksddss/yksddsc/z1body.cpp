#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <algorithm>

#include "dds/dds.hpp"
#include "nubotddsmsg.hpp"

#define TOTAL_MOTOR_NUMBER 48
#define ARM_MOTOR_NUMBER 12 //双臂的电机数
#define BODY_MOTOR_NUMBER 6 //双臂的电机数

//这里以后考虑参数传递或者文件配置主题名称
#define ARMCMDTOPIC "/nubot/z1/armmotorcmds"
#define LEGCMDTOPIC "/nubot/z1/legmotorcmds"

#define ARMSTATETOPIC "/nubot/z1/armmotorstates"
#define LEGSTATETOPIC "/nubot/z1/legmotorstates"

#define BODYCMDTOPIC "/nubot/z1/bodymotorcmds"
#define BODYSTATETOPIC "/nubot/z1/bodymotorstates"

#define WHOLEBODYCMDTOPIC  "/nubot/z1/wholebodymotorcmds"
#define WHOLEBODYSTATETOPIC  "/nubot/z1/wholebodymotorstates"

using namespace org::eclipse::cyclonedds;
using namespace nubotddsmsg::hr;

typedef struct {
    int mode;
    double pos_, vel_, tau_; //当前的位置rad、速度rad/s、力矩N.m
    double pos_des_, vel_des_, kp_, kd_, ff_; //期望的位置、速度、比例、积分、力矩N.m
    // mfn -> 新加
    int error_;
    double temperature_, mos_temperature_;
} YKSMotorData;

YKSMotorData my_motor_data[TOTAL_MOTOR_NUMBER];
// YKSMotorData arm_motor_data[ARM_MOTOR_NUMBER];

//用于响应ctrl+c退出
std::atomic<bool> quit(false);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nReceived Ctrl+C, exiting gracefully..." << std::endl;
        quit = true; // 设置退出标志
    }
}

void DDS_Get_Z1_5_WB_Motor_States(const int motor_num, const motorstates &states, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机状态数据，然后传给main函数当中的全局数组
    const int available_num = std::min<int>(motor_num, states.states().size());
    for (int i = 0; i < available_num; i++) {
        motor_cmds[i].mode = states.states()[i].mode();
        motor_cmds[i].pos_ = states.states()[i].pos();
        motor_cmds[i].vel_ = states.states()[i].vel();
        motor_cmds[i].tau_ = states.states()[i].tau();
        motor_cmds[i].error_ = states.states()[i].error();
        motor_cmds[i].temperature_ = states.states()[i].tem();
        motor_cmds[i].mos_temperature_ = states.states()[i].mos_tem();
    }
}
void DDS_Get_ARM_Motor_States(const int motor_num, const motorstates &states, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机状态数据，然后传给main函数当中的全局数组
    const int available_num = std::min<int>(motor_num, states.states().size());
    for (int i = 0; i < available_num; i++) {
        motor_cmds[i+12].mode = states.states()[i].mode();
        motor_cmds[i+12].pos_ = states.states()[i].pos();
        motor_cmds[i+12].vel_ = states.states()[i].vel();
        motor_cmds[i+12].tau_ = states.states()[i].tau();
        motor_cmds[i+12].error_ = states.states()[i].error();
        motor_cmds[i+12].temperature_ = states.states()[i].tem();
        motor_cmds[i+12].mos_temperature_ = states.states()[i].mos_tem();
    }
}

void DDS_Get_BODY_Motor_States(const int motor_num, const motorstates &states, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机状态数据，然后传给main函数当中的全局数组
    const int available_num = std::min<int>(motor_num, states.states().size());
    for (int i = 0; i < available_num; i++) {
        motor_cmds[i+24].mode = states.states()[i].mode();
        motor_cmds[i+24].pos_ = states.states()[i].pos();
        motor_cmds[i+24].vel_ = states.states()[i].vel();
        motor_cmds[i+24].tau_ = states.states()[i].tau();
        motor_cmds[i+24].error_ = states.states()[i].error();
        motor_cmds[i+24].temperature_ = states.states()[i].tem();
        motor_cmds[i+24].mos_temperature_ = states.states()[i].mos_tem();
    }
}
void DDS_SUB(dds::sub::DataReader<motorstates> &Reader, dds::sub::LoanedSamples<motorstates> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const motorstates &states = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                if (states.level() == 3) {
                    DDS_Get_Z1_5_WB_Motor_States(TOTAL_MOTOR_NUMBER, states, my_motor_data);
                }
                else if (states.level() == 1) {
                    DDS_Get_ARM_Motor_States(ARM_MOTOR_NUMBER, states, my_motor_data);
                }
                else if (states.level() == 2) {
                    DDS_Get_BODY_Motor_States(BODY_MOTOR_NUMBER, states, my_motor_data);
                }
            }
        }
    }
}

void DDS_Pub_Z1_5_WB_Motor_Cmds(motorcmds &cmds, dds::pub::DataWriter<motorcmds> &writer,
                                const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    int motor_num = TOTAL_MOTOR_NUMBER;
    cmds.level(3);
    for (int i = 0; i < motor_num; i++) {
        auto &state = cmds.cmds()[i];
        state.mode(motor_data_[i].mode);
        state.pos(motor_data_[i].pos_des_);
        state.vel(motor_data_[i].vel_des_);
        state.tau(motor_data_[i].ff_);
        state.kp(motor_data_[i].kp_);
        state.kd(motor_data_[i].kd_);
    }
    writer.write(cmds);
}

void DDS_Pub_ARM_Motor_Cmds(motorcmds &cmds, dds::pub::DataWriter<motorcmds> &writer,
                                const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    int motor_num = ARM_MOTOR_NUMBER;
    cmds.level(1);
    for (int i = 0; i < motor_num; i++) {
        auto &state = cmds.cmds()[i];
        state.mode(motor_data_[i+12].mode);
        state.pos(motor_data_[i+12].pos_des_);
        state.vel(motor_data_[i+12].vel_des_);
        state.tau(motor_data_[i+12].ff_);
        state.kp(motor_data_[i+12].kp_);
        state.kd(motor_data_[i+12].kd_);
    }
    writer.write(cmds);
}
void DDS_Pub_BODY_Motor_Cmds(motorcmds &cmds, dds::pub::DataWriter<motorcmds> &writer,
                                const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    int motor_num = BODY_MOTOR_NUMBER;
    cmds.level(1);
    for (int i = 0; i < motor_num; i++) {
        auto &state = cmds.cmds()[i];
        state.mode(motor_data_[i+24].mode);
        state.pos(motor_data_[i+24].pos_des_);
        state.vel(motor_data_[i+24].vel_des_);
        state.tau(motor_data_[i+24].ff_);
        state.kp(motor_data_[i+24].kp_);
        state.kd(motor_data_[i+24].kd_);
    }
    writer.write(cmds);
}
void setArmYKSSquatControl(int index, int mode, double pos, double vel, double tau, double kp, double kd) {
    // 检查索引是否在有效范围内 (12 <= arm_index < 24)
    if (index < 12 || index >= 24) {
        throw std::out_of_range("Invalid motor index. Must be between 12 and 23 inclusive.");
    }

    // 设置电机命令
    my_motor_data[index].mode = mode;
    my_motor_data[index].pos_des_ = pos;
    my_motor_data[index].vel_des_ = vel;
    my_motor_data[index].ff_ = tau;
    my_motor_data[index].kp_ = kp;
    my_motor_data[index].kd_ = kd;
}
void setBodyYKSSquatControl(int index, int mode, double pos, double vel, double tau, double kp, double kd) {
    // 检查索引是否在有效范围内 (12 <= arm_index < 24)
    if (index < 24 || index >= 30) {
        throw std::out_of_range("Invalid motor index. Must be between 12 and 23 inclusive.");
    }

    // 设置电机命令
    my_motor_data[index].mode = mode;
    my_motor_data[index].pos_des_ = pos;
    my_motor_data[index].vel_des_ = vel;
    my_motor_data[index].ff_ = tau;
    my_motor_data[index].kp_ = kp;
    my_motor_data[index].kd_ = kd;
}

int main() {
    // 注册信号处理函数
    std::signal(SIGINT, signal_handler);

    dds::domain::DomainParticipant participant(0);
    if (participant == dds::core::null) {
        std::cerr << "Failed to create participant!" << std::endl;
        return -1;
    }

    // // z1_5_wb订阅 ========================================================================================
    // //定义z1_5_wb订阅者话题
    // dds::topic::Topic<motorstates> z1_5_wb_topicsub(participant, WHOLEBODYSTATETOPIC);
    // //定义z1_5_wb订阅者话题Qos
    // dds::sub::Subscriber z1_5_wb_Subscriber(participant);
    // dds::sub::qos::DataReaderQos z1_5_wb_ReadQos = z1_5_wb_Subscriber.default_datareader_qos();
    // z1_5_wb_ReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
    //         << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
    //         << dds::core::policy::History::KeepLast(5); //保留近5条消息
    //
    // //定义z1_5_wbReader
    // dds::sub::DataReader<motorstates> z1_5_wb_Reader(z1_5_wb_Subscriber, z1_5_wb_topicsub, z1_5_wb_ReadQos);
    // std::cout << "=== [z1_5_wb subscriber] get ready! " << std::endl;
    //
    // // // z1_5_wb发布 =======================================================================================
    // // // 定义z1_5_wb发布者话题
    // dds::topic::qos::TopicQos z1_5_wb_topicpubQos;
    // z1_5_wb_topicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
    //         << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
    //         << dds::core::policy::History::KeepLast(5); //保留近5条消息
    // dds::topic::Topic<motorcmds> z1_5_wb_topicpub(participant, WHOLEBODYCMDTOPIC);
    // // 创建 Publisher 和 DataWriter
    // dds::pub::Publisher z1_5_wb_Publisher(participant);
    // dds::pub::qos::DataWriterQos z1_5_wb_writerQos(z1_5_wb_topicpubQos); // datawriter的qos应当继承自topic的qos
    // dds::pub::DataWriter<motorcmds> z1_5_wb_Writer(z1_5_wb_Publisher, z1_5_wb_topicpub, z1_5_wb_writerQos);
    // std::cout << "=== [z1_5_wb publisher] get ready! " << std::endl;

    // 上肢订阅 ========================================================================================
    //定义上肢订阅者话题
    dds::topic::Topic<motorstates> armtopicsub(participant, ARMSTATETOPIC);
    //定义下肢订阅者话题Qos
    dds::sub::Subscriber armSubscriber(participant);
    dds::sub::qos::DataReaderQos armReadQos = armSubscriber.default_datareader_qos();
    armReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义上肢Reader
    dds::sub::DataReader<motorstates> armReader(armSubscriber, armtopicsub, armReadQos);
    std::cout << "=== [Arm subscriber] get ready! " << std::endl;

    // // 上肢发布 =======================================================================================
    // // 定义上肢发布者话题
    dds::topic::qos::TopicQos armtopicpubQos;
    armtopicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息
    dds::topic::Topic<motorcmds> armtopicpub(participant, ARMCMDTOPIC);
    // 创建 Publisher 和 DataWriter
    dds::pub::Publisher armPublisher(participant);
    dds::pub::qos::DataWriterQos armwriterQos(armtopicpubQos); // datawriter的qos应当继承自topic的qos
    dds::pub::DataWriter<motorcmds> armWriter(armPublisher, armtopicpub, armwriterQos);
    std::cout << "=== [arm publisher] get ready! " << std::endl;

    // 躯干订阅 ========================================================================================
    //定义躯干订阅者话题
    dds::topic::Topic<motorstates> bodytopicsub(participant, BODYSTATETOPIC);
    //定义躯干订阅者话题Qos
    dds::sub::Subscriber bodySubscriber(participant);
    dds::sub::qos::DataReaderQos bodyReadQos = bodySubscriber.default_datareader_qos();
    bodyReadQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息

    //定义躯干Reader
    dds::sub::DataReader<motorstates> bodyReader(bodySubscriber, bodytopicsub, bodyReadQos);
    std::cout << "=== [body subscriber] get ready! " << std::endl;

    // // 躯干发布 =======================================================================================
    // // 定义躯干发布者话题
    dds::topic::qos::TopicQos bodytopicpubQos;
    bodytopicpubQos << dds::core::policy::Reliability::BestEffort() //尽力传输，只发一次
            << dds::core::policy::Durability::Volatile() //持久化，比如订阅者后加入，则不接受历史信息，只接收自加入以来的消息
            << dds::core::policy::History::KeepLast(5); //保留近5条消息
    dds::topic::Topic<motorcmds> bodytopicpub(participant, BODYCMDTOPIC);
    // 创建 Publisher 和 DataWriter
    dds::pub::Publisher bodyPublisher(participant);
    dds::pub::qos::DataWriterQos bodywriterQos(bodytopicpubQos); // datawriter的qos应当继承自topic的qos
    dds::pub::DataWriter<motorcmds> bodyWriter(bodyPublisher, bodytopicpub, bodywriterQos);
    std::cout << "=== [body publisher] get ready! " << std::endl;

    // motorcmds z1_5_wb_Cmds;
    // z1_5_wb_Cmds.level(3); // 设置为z1_5_wb
    // z1_5_wb_Cmds.cmds().resize(TOTAL_MOTOR_NUMBER);

    motorcmds arm_Cmds;
    arm_Cmds.level(1); // 设置为上肢
    arm_Cmds.cmds().resize(ARM_MOTOR_NUMBER);

    motorcmds body_Cmds;
    body_Cmds.level(1); // 设置为上肢
    body_Cmds.cmds().resize(BODY_MOTOR_NUMBER);
    //初始化
    // for (int i = 0; i < TOTAL_MOTOR_NUMBER; ++i) {
    //     auto &cmd = z1_5_wb_Cmds.cmds()[i];
    //
    //     cmd.mode(0);
    //     cmd.index(i);
    //     cmd.pos(0);
    //     cmd.vel(0);
    //     cmd.tau(0);
    //     cmd.kp(0);
    //     cmd.kd(0);
    // }
    for (int i = 0; i < ARM_MOTOR_NUMBER; ++i) {
        auto &cmd = arm_Cmds.cmds()[i];

        cmd.mode(0);
        cmd.index(i);
        cmd.pos(0);
        cmd.vel(0);
        cmd.tau(0);
        cmd.kp(0);
        cmd.kd(0);
    }
    for (int i = 0; i < BODY_MOTOR_NUMBER; ++i) {
        auto &cmd = body_Cmds.cmds()[i];

        cmd.mode(0);
        cmd.index(i);
        cmd.pos(0);
        cmd.vel(0);
        cmd.tau(0);
        cmd.kp(0);
        cmd.kd(0);
    }
    // dds::sub::LoanedSamples<motorstates> samples_z1_5_wb;
    dds::sub::LoanedSamples<motorstates> samples_arm;
    dds::sub::LoanedSamples<motorstates> samples_body;

    setArmYKSSquatControl(13, 0, 0, 0, 0, 0, 10);//12-23
    setBodyYKSSquatControl(25, 0, 0, 0, 0, 0, 10);//24-29
    while (!quit) {
        /////////////////////////////////////////////////////////////////////////////////////////////

        DDS_SUB(armReader, samples_arm); //
        DDS_SUB(bodyReader, samples_body);

        DDS_Pub_ARM_Motor_Cmds(arm_Cmds, armWriter, my_motor_data);
        DDS_Pub_BODY_Motor_Cmds(body_Cmds, bodyWriter, my_motor_data);

        std::cout << my_motor_data[1].mode << " " << my_motor_data[2].pos_ << " " << my_motor_data[2].vel_ << " " <<
                my_motor_data[2].tau_ << " " << my_motor_data[2].error_ << " " << my_motor_data[2].temperature_ <<
                " "
                << my_motor_data[2].mos_temperature_ << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return EXIT_FAILURE;
}
