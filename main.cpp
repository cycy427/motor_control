#include "SBusReceiver.h"
#include "JoyStickHandler.h"
#include "Z1Legs.h"
#include "BmsHandler.h"
#include "SocketReceiver.h"
#include "SocketSender.h"
#include "dds/dds.hpp"
#include "nubotddsmsg.hpp"

#define DDS  //如果想要使用Socket通信，那么就注释，如果想使用DDS通信，那么请取消注释

//这里以后考虑参数传递或者文件配置主题名称
#define ARMCMDTOPIC "/nubot/z1/armmotorcmds"
#define LEGCMDTOPIC "/nubot/z1/legmotorcmds"

#define ARMSTATETOPIC "/nubot/z1/armmotorstates"
#define LEGSTATETOPIC "/nubot/z1/legmotorstates"

#define BODYCMDTOPIC "/nubot/z1/bodymotorcmds"
#define BODYSTATETOPIC "/nubot/z1/bodymotorstates"

using namespace org::eclipse::cyclonedds;
using namespace nubotddsmsg::hr;


YKSMotorData my_motor_data[Z1_NUM_MOTOR];

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

void DDS_Get_Leg_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    for (int i = 0; i < motor_num; i++) {
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
    for (int i = 0; i < motor_num; i++) {
        motor_cmds[i+12].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i+12].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i+12].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i+12].mode = cmds.cmds()[i].mode();
        motor_cmds[i+12].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i+12].kd_ = cmds.cmds()[i].kd();
    }
}

void DDS_Get_Body_Motor_Cmds(const int motor_num, const motorcmds &cmds, YKSMotorData *motor_cmds) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    for (int i = 0; i < motor_num; i++) {
        motor_cmds[i+24].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[i+24].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[i+24].ff_ = cmds.cmds()[i].tau();
        motor_cmds[i+24].mode = cmds.cmds()[i].mode();
        motor_cmds[i+24].kp_ = cmds.cmds()[i].kp();
        motor_cmds[i+24].kd_ = cmds.cmds()[i].kd();
    }
}

void DDS_Pub_Arm_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                        const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    int motor_num = ARM_MOTOR_NUMBER;
    for (int i = 0; i < motor_num; i++) {
        auto &state = states.states()[i];
        state.mode(motor_data_[i+12].mode);
        state.index(i + 12);
        state.pos(motor_data_[i+12].pos_);
        state.vel(motor_data_[i+12].vel_);
        state.cur(motor_data_[i+12].tau_);
        state.tau(motor_data_[i+12].tau_);
        state.tau_raw(motor_data_[i+12].tau_);
        state.error(motor_data_[i+12].error_);
        state.tem(motor_data_[i+12].temperature_);
        state.mos_tem(motor_data_[i+12].mos_temperature_);
    }
    writer.write(states);
}

void DDS_Pub_Leg_Motor_Data( motorstates &states, dds::pub::DataWriter<motorstates> &writer,
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

void DDS_Pub_Body_Motor_Data( motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                        const YKSMotorData *motor_data_) {
    // 拿到所有DDS传过来的电机指令数据，然后传给main函数当中的全局数组，通过电机数量可以区分到底是上肢还是下肢的指令
    int motor_num = BODY_MOTOR_NUMBER;

    for (int i = 0; i < motor_num; i++) {
        auto &state = states.states()[i];
        state.mode(motor_data_[i+24].mode);
        state.index(i + 24);
        state.pos(motor_data_[i+24].pos_);
        state.vel(motor_data_[i+24].vel_);
        state.cur(motor_data_[i+24].tau_);
        state.tau(motor_data_[i+24].tau_);
        state.tau_raw(motor_data_[i+24].tau_);
        state.error(motor_data_[i+24].error_);
        state.tem(motor_data_[i+24].temperature_);
        state.mos_tem(motor_data_[i+24].mos_temperature_);
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

int main() {
    //////////////////////////////////////////////////////////////////////////////////////////////////
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
    //////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////

    if (bool Ethernet_Status = CAT_Init("enp3s0"); !Ethernet_Status) { exit(1); } //如果初始化失败，则直接退出程序
    auto joystick_device = "/dev/input/js0";
    // auto battery = "/dev/ttyUSB1";
    const auto joystick_handler = std::make_shared<JoyStickHandler>(joystick_device);
    // const auto battery_handler = std::make_shared<BmsHandler>(battery);
    const SBusReceiver sbus_receiver("/dev/SBUS1");
    Z1Legs z1_legs;
    z1_legs.setJoyStickHandler(joystick_handler);
    // z1_legs.setBatteryHandler(battery_handler);

    SocketReceiver receiver(YKS_PORT);
    SocketSender sender("127.0.0.1",USR_PORT);
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

    dds::sub::LoanedSamples<motorcmds> samples_leg;
    dds::sub::LoanedSamples<motorcmds> samples_arm;
    dds::sub::LoanedSamples<motorcmds> samples_body;

    // double pos_pitch = 0;
    // double pos_roll = 0;
    while (true) {
        if (z1_legs.stop_) {
            break;
        }
#ifdef DDS
        /////////////////////////////////////////////////////////////////////////////////////////////
        //读取leg订阅的消息 ---------------------------------------------------------------------------
        DDS_Leg_SUB(legReader, samples_leg);
        /////////////////////////////////////////////////////////////////////////////////////////////
        //读取arm订阅的消息 -----------------------------------------------------------------------------
        DDS_Arm_SUB(armReader, samples_arm); //
        //读取body订阅的消息 -----------------------------------------------------------------------------
        DDS_Body_SUB(bodyReader, samples_body); //
        /////////////////////////////////////////////////////////////////////////////////////////////
#endif
#ifndef DDS
        //读取Socket通信的消息 -----------------------------------------------------------------------------
        receiver.getSocketMotorCMD(my_motor_data);
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

        ///////////////////////////////////////////////////////////////////////////////////////////
        ///将上肢电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        DDS_Pub_Arm_Motor_Data(armStates, armWriter, my_motor_data);
        ///将下肢电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        DDS_Pub_Leg_Motor_Data(legStates, legWriter, my_motor_data);
        ///将躯干电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        DDS_Pub_Body_Motor_Data(bodyStates, bodyWriter, my_motor_data);
        ///通过Socket将电机状态发送给用户端
        sender.sendSocketMotorData(my_motor_data); //通过Socket反馈电机当前的数据
        // SBusData data = sbus_receiver.getData();
        // SBusReceiver::print_data(data);
        // pos = data.ch[2] / 672.0 * 4;
        // JoystickState state = joystick_handler->getState();
        // JoyStickHandler::print_state(state);
        // squat_control(pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //读取周期为1ms
    }
    runThread.join(); //runThread.join(); 的主要功能是确保 main 函数在退出之前等待 runThread 线程完成其任务。
    // 这样做的目的是为了确保程序在退出前所有的后台任务都得到了正确地执行和清理，避免数据丢失或资源泄露。
    return 0;
}
