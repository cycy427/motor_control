#include "SBusReceiver.h"
#include "JoyStickHandler.h"
#include "Z1Legs.h"
#include "BmsHandler.h"
#include "SocketReceiver.h"
#include "SocketSender.h"
#include "dds/dds.hpp"
#include "nubotddsmsg.hpp"
//这里以后考虑参数传递或者文件配置主题名称
#define ARMCMDTOPIC "/nubot/z1/armmotorcmds"
#define LEGCMDTOPIC "/nubot/z1/legmotorcmds"

#define ARMSTATETOPIC "/nubot/z1/armmotorstates"
#define LEGSTATETOPIC "/nubot/z1/legmotorstates"

using namespace org::eclipse::cyclonedds;
using namespace nubotddsmsg::hr;


YKSMotorData my_motor_data[Z1_NUM_MOTOR];

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

int main() {
    if (bool Ethernet_Status = CAT_Init("enp5s0"); !Ethernet_Status) { exit(1); } //如果初始化失败，则直接退出程序
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

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////

    motorstates armStates;
    armStates.level(1); // 设置为上肢
    armStates.states().resize(14);
    //初始化
    for (int i = 0; i < 14; ++i) {
        auto &state = armStates.states()[i];

        state.mode(0);
        state.index(i + 1);
        state.pos(0);
        state.vel(0);
        state.cur(0);
        state.tau(0);
        state.tau_raw(0);
        state.error(0);
        state.tem(15);
        state.mos_tem(15);
    }

    motorstates legStates;
    legStates.level(0); // 设置为下肢
    legStates.states().resize(13);
    //初始化
    for (int i = 0; i < 13; ++i) {
        auto &state = legStates.states()[i];

        state.mode(0);
        state.index(i + 1);
        state.pos(0);
        state.vel(0);
        state.cur(0);
        state.tau(0);
        state.tau_raw(0);
        state.error(0);
        state.tem(15);
        state.mos_tem(15);
    }

    dds::sub::LoanedSamples<motorcmds> samples;


    // double pos_pitch = 0;
    // double pos_roll = 0;
    while (true) {
        if (z1_legs.stop_) {
            break;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        //读取leg订阅的消息 ---------------------------------------------------------------------------
        samples = legReader.take();
        if (samples.length() > 0) {
            dds::sub::LoanedSamples<motorcmds>::const_iterator sample_iter;
            for (sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
                const motorcmds &legcmds = sample_iter->data();
                const dds::sub::SampleInfo &info = sample_iter->info();
                if (info.valid()) {
                    std::cout << "legMotorcmds" << (int) legcmds.level() << "   pos:" << legcmds.cmds()[0].pos() <<
                            std::endl;
                    // 拿到所有下肢电机指令数据，调用z1legs类下发命令
                    // for (auto& cmd : legcmds.cmds()) {
                    //     std::cout << "  index:" << cmd.index()
                    //               << ": pos=" << cmd.pos()
                    //               << ", vel=" << cmd.vel()
                    //               << ", tau=" << cmd.tau() << std::endl;

                    ///////////////////////////////////////////////////////////
                    // do something here
                    // }
                }
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        //读取arm订阅的消息 -----------------------------------------------------------------------------
        samples = armReader.take();
        if (samples.length() > 0) {
            dds::sub::LoanedSamples<motorcmds>::const_iterator sample_iter;
            for (sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
                const motorcmds &armcmds = sample_iter->data();
                const dds::sub::SampleInfo &info = sample_iter->info();
                if (info.valid()) {
                    std::cout << "legMotorcmds" << (int) armcmds.level() << "   pos:" << armcmds.cmds()[0].pos() <<
                            std::endl;
                    //拿到所有下肢电机指令数据，调用z1arms类下发命令
                    // for (auto& cmd : armcmds.cmds()) {
                    //     std::cout << "  armMotorcmds " << cmd.index()
                    //               << ": pos=" << cmd.pos()
                    //               << ", vel=" << cmd.vel()
                    //               << ", tau=" << cmd.tau() << std::endl;

                    ////////////////////////////////////////////////////////////
                    /// do something here
                    // }
                }
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////
        //获取所有电机的状态


        ///////////////////////////////////////////////////////////////////////////////////////////
        ///将上肢电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        for (int i = 0; i < TI5_MOTOR_NUMBER; ++i) {
            auto &state = armStates.states()[i];

            state.mode(0);
            state.index(i);
            state.pos(0);
            state.vel(0);
            state.cur(0);
            state.tau(0);
            state.tau_raw(0);
            state.error(0);
            state.tem(15);
            state.mos_tem(15);
        }

        ///将下肢电机状态写入消息 啦啦啦啦啦啦啦啦啦啦啦
        for (int i = 0; i < YKS_MOTOR_NUMBER; ++i) {
            auto &state = legStates.states()[i];

            state.mode(0);
            state.index(i);
            state.pos(0);
            state.vel(0);
            state.cur(0);
            state.tau(0);
            state.tau_raw(0);
            state.error(0);
            state.tem(15);
            state.mos_tem(15);
        }

        armWriter.write(armStates); //发布消息
        legWriter.write(legStates); //发布消息
        // SBusData data = sbus_receiver.getData();
        // SBusReceiver::print_data(data);
        // pos = data.ch[2] / 672.0 * 4;
        // JoystickState state = joystick_handler->getState();
        // JoyStickHandler::print_state(state);
        // pos_roll = state.left_stick_x / 40000.0 * 10;
        // pos_pitch = state.left_stick_y / 40000.0 * 10;
        // printf("pos: %f\n", pos);

        // squat_control(pos);
        // my_motor_data[LeftAnklePitch].pos_des_ = pos_pitch; //设置电机目标位置
        // my_motor_data[LeftAnkleRoll].pos_des_ = pos_roll; //设置电机目标位置
        // my_motor_data[RightAnklePitch].pos_des_ = pos_pitch; //设置电机目标位置
        // my_motor_data[RightAnkleRoll].pos_des_ = pos_roll; //设置电机目标位置
        // my_motor_data[LeftAnklePitch].ff_ = pos_pitch; //设置电机目标位置
        // my_motor_data[LeftAnkleRoll].ff_ = pos_roll; //设置电机目标位置
        // my_motor_data[RightAnklePitch].ff_ = pos_pitch; //设置电机目标位置
        // my_motor_data[RightAnkleRoll].ff_ = pos_roll; //设置电机目标位置
        // my_motor_data[LeftKnee].ff_ = -pos_roll; //设置电机目标位置
        // my_motor_data[RightKnee].ff_ = pos_roll; //设置电机目标位置
        // my_motor_data[Z1JointIndex::LeftAnkleRoll].ff_ = pos_roll; //设置电机目标位置
        receiver.getSocketMotorCMD(my_motor_data);
        z1_legs.setMotorKpKd(my_motor_data); //专门设置电机KP、KD值，调用了这个函数之后就会将原来设置在Z1legs类里面的默认KP、KD值覆盖掉
        z1_legs.setMotorCommand(my_motor_data); //设置电机指令
        z1_legs.getMotorData(my_motor_data); //获取电机数据
        sender.sendSocketMotorData(my_motor_data); //通过Socket反馈电机当前的数据
        // printf("ld_pitch: %f  ld_roll: %f l_pitch: %f  l_roll: %f rd_pitch: %f  rd_roll: %f r_pitch: %f  r_roll: %f \n", pos_pitch, pos_roll,
        //        my_motor_data[LeftAnklePitch].tau_, my_motor_data[LeftAnkleRoll].tau_,pos_pitch, pos_roll,
        //        my_motor_data[RightAnklePitch].tau_, my_motor_data[RightAnkleRoll].tau_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //读取周期为1ms
    }
    runThread.join(); //runThread.join(); 的主要功能是确保 main 函数在退出之前等待 runThread 线程完成其任务。
    // 这样做的目的是为了确保程序在退出前所有的后台任务都得到了正确地执行和清理，避免数据丢失或资源泄露。
    return 0;
}
