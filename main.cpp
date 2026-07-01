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
#include <csignal>
#include <unistd.h>
#include <cstring>
#define DDS

// TODO: 主题名称后续改为参数或配置文件。
#define ARMCMDTOPIC "/nubot/z1/armmotorcmds"
#define LEGCMDTOPIC "/nubot/z1/legmotorcmds"

#define ARMSTATETOPIC "/nubot/z1/armmotorstates"
#define LEGSTATETOPIC "/nubot/z1/legmotorstates"

#define BODYCMDTOPIC "/nubot/z1/bodymotorcmds"
#define BODYSTATETOPIC "/nubot/z1/bodymotorstates"

#define WHOLEBODYCMDTOPIC "/nubot/z1/wholebodymotorcmds"
#define WHOLEBODYSTATETOPIC "/nubot/z1/wholebodymotorstates"

#define SBUSSTATETOPIC  "/nubot/z1/sbusstates"

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

std::mutex imu_mutex;
std::condition_variable imu_cv;
std::mutex hcmd_mutex;
std::condition_variable hcmd_cv;
std::mutex logic_mutex;
std::condition_variable logic_cv;
YKSMotorData my_motor_data[Z1_NUM_MOTOR];
bool is_imu_run = false;
bool is_hcmd_run = false;
bool is_logic_run = false;
bool is_bms_run = false;

std::atomic<bool> stop_thread(false);
volatile sig_atomic_t stop_flag = false;
void signalHandler(int signal) {
    if (signal == SIGINT) {
        stop_flag = true;
        const char *msg = "\nSIGINT received, preparing to exit...\n";
        write(STDOUT_FILENO, msg, std::strlen(msg));
    }
}

// 下肢蹲起测试函数。
void squat_control(const float pos) {
    my_motor_data[Z1JointIndex::LeftHipYaw].pos_des_ = 0;
    my_motor_data[Z1JointIndex::LeftHipPitch].pos_des_ = -pos * 0.25;
    my_motor_data[Z1JointIndex::LeftHipRoll].pos_des_ = 0;
    my_motor_data[Z1JointIndex::LeftKnee].pos_des_ = pos * 0.7;
    my_motor_data[Z1JointIndex::LeftAnkleA].pos_des_ = -pos * 0.1;
    my_motor_data[Z1JointIndex::LeftAnkleB].pos_des_ = pos * 0.1;

    my_motor_data[Z1JointIndex::RightHipYaw].pos_des_ = 0;
    my_motor_data[Z1JointIndex::RightHipPitch].pos_des_ = pos * 0.25;
    my_motor_data[Z1JointIndex::RightHipRoll].pos_des_ = 0;
    my_motor_data[Z1JointIndex::RightKnee].pos_des_ = -pos * 0.7;
    my_motor_data[Z1JointIndex::RightAnkleA].pos_des_ = pos * 0.1;
    my_motor_data[Z1JointIndex::RightAnkleB].pos_des_ = -pos * 0.1;
}

// 清空待发送电机指令。
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

void DDS_Get_Arm_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
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

void DDS_Get_Body_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
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

void DDS_Get_WholeBody_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    const int available_num = std::min<int>(motor_num, cmds.cmds().size());
    for (int i = 0; i < available_num; i++) {
        const int gid = kAllActiveIds[i];
        motor_cmds[gid].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[gid].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[gid].ff_ = cmds.cmds()[i].tau();
        motor_cmds[gid].mode = cmds.cmds()[i].mode();
        motor_cmds[gid].kp_ = cmds.cmds()[i].kp();
        motor_cmds[gid].kd_ = cmds.cmds()[i].kd();
    }
}

void DDS_Pub_Arm_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
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

void DDS_Pub_Leg_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
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

void DDS_Pub_WholeBody_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                                  const YKSMotorData *motor_data_) {
    int motor_num = ACTIVE_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        const int gid = kAllActiveIds[i];
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

void DDS_Pub_Sbus_Data(sbusdata &states, dds::pub::DataWriter<sbusdata> &writer,
                       const SBusData *sbus_data_) {
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

void DDS_WholeBody_SUB(dds::sub::DataReader<motorcmds> &Reader, dds::sub::LoanedSamples<motorcmds> &samples) {
    samples = Reader.take();
    if (samples.length() > 0) {
        for (auto sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
            const motorcmds &cmds = sample_iter->data();
            const dds::sub::SampleInfo &info = sample_iter->info();
            if (info.valid()) {
                DDS_Get_WholeBody_Motor_Cmds(ACTIVE_MOTOR_NUMBER, cmds, my_motor_data);
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
            }
        }
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
            }
        }
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
            }
        }
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
            }
        }
    }
}

int main() {
    std::signal(SIGINT, signalHandler);

    setenv("CYCLONEDDS_URI", "file:///home/wcy/humanoid_ws/humanoid_proj/z1_rl/dds_helper/dds_config/NetworkInterface.xml", 1);
    const char *uri = getenv("CYCLONEDDS_URI");
    std::cout << "Active URI: " << (uri ? uri : "NULL") << std::endl;

    dds::domain::DomainParticipant participant(0);
    if (participant == dds::core::null) {
        std::cerr << "Failed to create participant!" << std::endl;
        return -1;
    }

    // DDS subscribers.
    dds::topic::Topic<motorcmds> legtopicsub(participant, LEGCMDTOPIC);
    dds::sub::Subscriber legSubscriber(participant);
    dds::sub::qos::DataReaderQos legReadQos = legSubscriber.default_datareader_qos();
    legReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<motorcmds> legReader(legSubscriber, legtopicsub, legReadQos);
    std::cout << "=== [Leg subscriber] get ready! " << std::endl;

    dds::topic::Topic<motorcmds> armtopicsub(participant, ARMCMDTOPIC);
    dds::sub::Subscriber armSubscriber(participant);
    dds::sub::qos::DataReaderQos armReadQos = armSubscriber.default_datareader_qos();
    armReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<motorcmds> armReader(armSubscriber, armtopicsub, armReadQos);
    std::cout << "=== [Arm subscriber] get ready! " << std::endl;

    dds::topic::Topic<motorcmds> bodytopicsub(participant, BODYCMDTOPIC);
    dds::sub::Subscriber bodySubscriber(participant);
    dds::sub::qos::DataReaderQos bodyReadQos = bodySubscriber.default_datareader_qos();
    bodyReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<motorcmds> bodyReader(bodySubscriber, bodytopicsub, bodyReadQos);
    std::cout << "=== [body subscriber] get ready! " << std::endl;

    dds::topic::Topic<imudata> imu_topicsub(participant, IMUPUBDATATOPIC);
    dds::sub::Subscriber imu_Subscriber(participant);
    dds::sub::qos::DataReaderQos imu_ReadQos = imu_Subscriber.default_datareader_qos();
    imu_ReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<imudata> imu_Reader(imu_Subscriber, imu_topicsub, imu_ReadQos);
    std::cout << "=== [imu subscriber] get ready! " << std::endl;

    dds::topic::Topic<hcmddata> hcmd_topicsub(participant, HCMDPUBDATATOPIC);
    dds::sub::Subscriber hcmd_Subscriber(participant);
    dds::sub::qos::DataReaderQos hcmd_ReadQos = hcmd_Subscriber.default_datareader_qos();
    hcmd_ReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<hcmddata> hcmd_Reader(hcmd_Subscriber, hcmd_topicsub, hcmd_ReadQos);
    std::cout << "=== [hcmd subscriber] get ready! " << std::endl;

    dds::topic::Topic<logicdata> logic_topicsub(participant, LOGICPUBDATATOPIC);
    dds::sub::Subscriber logic_Subscriber(participant);
    dds::sub::qos::DataReaderQos logic_ReadQos = logic_Subscriber.default_datareader_qos();
    logic_ReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<logicdata> logic_Reader(logic_Subscriber, logic_topicsub, logic_ReadQos);
    std::cout << "=== [logic subscriber] get ready! " << std::endl;

    dds::topic::Topic<bmsdata_short> bms_topicsub(participant, BMSPUBDATATOPIC);
    dds::sub::Subscriber bms_Subscriber(participant);
    dds::sub::qos::DataReaderQos bms_ReadQos = bms_Subscriber.default_datareader_qos();
    bms_ReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<bmsdata_short> bms_Reader(bms_Subscriber, bms_topicsub, bms_ReadQos);
    std::cout << "=== [bms subscriber] get ready! " << std::endl;

    dds::topic::Topic<motorcmds> wholebody_topicsub(participant, WHOLEBODYCMDTOPIC);
    dds::sub::Subscriber wholebody_Subscriber(participant);
    dds::sub::qos::DataReaderQos wholebody_ReadQos = wholebody_Subscriber.default_datareader_qos();
    wholebody_ReadQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);

    dds::sub::DataReader<motorcmds> wholebody_Reader(wholebody_Subscriber, wholebody_topicsub, wholebody_ReadQos);
    std::cout << "=== [wholebody subscriber] get ready! " << std::endl;

    // DDS publishers.
    dds::topic::qos::TopicQos legtopicpubQos;
    legtopicpubQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);
    dds::topic::Topic<motorstates> legtopicpub(participant, LEGSTATETOPIC);
    dds::pub::Publisher legPublisher(participant);
    dds::pub::qos::DataWriterQos legwriterQos(legtopicpubQos);
    dds::pub::DataWriter<motorstates> legWriter(legPublisher, legtopicpub, legwriterQos);
    std::cout << "=== [Leg publisher] get ready! " << std::endl;

    dds::topic::qos::TopicQos armtopicpubQos;
    armtopicpubQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);
    dds::topic::Topic<motorstates> armtopicpub(participant, ARMSTATETOPIC);
    dds::pub::Publisher armPublisher(participant);
    dds::pub::qos::DataWriterQos armwriterQos(armtopicpubQos);
    dds::pub::DataWriter<motorstates> armWriter(armPublisher, armtopicpub, armwriterQos);
    std::cout << "=== [arm publisher] get ready! " << std::endl;

    dds::topic::qos::TopicQos bodytopicpubQos;
    bodytopicpubQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);
    dds::topic::Topic<motorstates> bodytopicpub(participant, BODYSTATETOPIC);
    dds::pub::Publisher bodyPublisher(participant);
    dds::pub::qos::DataWriterQos bodywriterQos(bodytopicpubQos);
    dds::pub::DataWriter<motorstates> bodyWriter(bodyPublisher, bodytopicpub, bodywriterQos);
    std::cout << "=== [body publisher] get ready! " << std::endl;

    dds::topic::qos::TopicQos wholebody_topicpubQos;
    wholebody_topicpubQos << dds::core::policy::Reliability::BestEffort()
            << dds::core::policy::Durability::Volatile()
            << dds::core::policy::History::KeepLast(5);
    dds::topic::Topic<motorstates> wholebody_topicpub(participant, WHOLEBODYSTATETOPIC);
    dds::pub::Publisher wholebody_Publisher(participant);
    dds::pub::qos::DataWriterQos wholebody_writerQos(wholebody_topicpubQos);
    dds::pub::DataWriter<motorstates> wholebody_Writer(wholebody_Publisher, wholebody_topicpub, wholebody_writerQos);
    std::cout << "=== [wholebody publisher] get ready! " << std::endl;

    if (bool Ethernet_Status = CAT_Init("enx6c1ff7bb1b4d"); !Ethernet_Status) { exit(1); }
    Z1Legs z1_legs;

    motorstates armStates;
    armStates.level(1);
    armStates.states().resize(ARM_MOTOR_NUMBER);

    motorstates legStates;
    legStates.level(0);
    legStates.states().resize(LEG_MOTOR_NUMBER);

    motorstates bodyStates;
    bodyStates.level(2);
    bodyStates.states().resize(BODY_MOTOR_NUMBER);

    motorstates wholebody_States;
    wholebody_States.level(3);
    wholebody_States.states().resize(ACTIVE_MOTOR_NUMBER);

    sbusdata sbus_States;


    dds::sub::LoanedSamples<motorcmds> samples_leg;
    dds::sub::LoanedSamples<motorcmds> samples_arm;
    dds::sub::LoanedSamples<motorcmds> samples_body;
    dds::sub::LoanedSamples<motorcmds> samples_wholebody;
    dds::sub::LoanedSamples<imudata> samples_imu;
    dds::sub::LoanedSamples<hcmddata> samples_hcmd;
    dds::sub::LoanedSamples<logicdata> samples_logic;
    dds::sub::LoanedSamples<bmsdata_short> samples_bms;

    // 等待 IMU 和逻辑模块上线。
    while (!stop_flag) {
        DDS_IMU_SUB(imu_Reader, samples_imu);
        z1_legs.getIMUFlag(is_imu_run);

        DDS_HCMD_SUB(hcmd_Reader, samples_hcmd);
        z1_legs.getHcmdFlag(is_hcmd_run);

        DDS_LOGIC_SUB(logic_Reader, samples_logic);
        z1_legs.getLogicFlag(is_logic_run);

        DDS_BMS_SUB(bms_Reader, samples_bms);
        z1_legs.getBmsFlag(is_bms_run);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if(true){
        // if (is_logic_run == true && is_imu_run == true ) {
            break;
        }
        //printf("Error!!!!!!!!!! Please connect imu and switch!!!!!!!!!!");
    }

    // 主循环：接收指令、下发电机、读取状态并发布反馈。
    while (!stop_flag) {
#ifdef DDS
        DDS_Leg_SUB(legReader, samples_leg);
        DDS_Arm_SUB(armReader, samples_arm);
        DDS_Body_SUB(bodyReader, samples_body);
        DDS_WholeBody_SUB(wholebody_Reader, samples_wholebody);
#endif
#ifndef DDS
        // Socket模式入口：receiver.getSocketMotorCMD(my_motor_data);
#endif
        // 外部指令会覆盖Z1Legs内部默认KP/KD。
        z1_legs.setMotorKpKd(my_motor_data);
        z1_legs.setMotorCommand(my_motor_data);
        z1_legs.getMotorData(my_motor_data);

        DDS_Pub_Arm_Motor_Data(armStates, armWriter, my_motor_data);
        DDS_Pub_Leg_Motor_Data(legStates, legWriter, my_motor_data);
        DDS_Pub_Body_Motor_Data(bodyStates, bodyWriter, my_motor_data);
        DDS_Pub_WholeBody_Motor_Data(wholebody_States, wholebody_Writer, my_motor_data);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // 退出前清零指令并等待EtherCAT线程结束。
    zero_out();
    z1_legs.setMotorKpKd(my_motor_data);
    z1_legs.setMotorCommand(my_motor_data);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    stop_thread = true;
    if (runThread.joinable()) {
        runThread.join();
        printf("[EtherCAT] Stopping runImpl accomplished.\n");
    }
    pthread_join(*checkThread, nullptr);
    printf("[EtherCAT] Stopping ethercat_check accomplished.\n");

    std::signal(SIGINT, SIG_DFL);
    return 0;
}
