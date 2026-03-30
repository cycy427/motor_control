#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include "dds/dds.hpp"
#include "nubotddsmsg.hpp"

//这里以后考虑参数传递或者文件配置主题名称
#define ARMCMDTOPIC "/nubot/z1/armmotorcmds"
#define LEGCMDTOPIC "/nubot/z1/legmotorcmds"

#define ARMSTATETOPIC "/nubot/z1/armmotorstates"
#define LEGSTATETOPIC "/nubot/z1/legmotorstates"

using namespace org::eclipse::cyclonedds;
using namespace nubotddsmsg::hr;

//用于响应ctrl+c退出
std::atomic<bool> quit(false);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nReceived Ctrl+C, exiting gracefully..." << std::endl;
        quit = true;  // 设置退出标志
    }
}

int main() {
    // 注册信号处理函数
    std::signal(SIGINT, signal_handler);

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
    dds::sub::DataReader<motorcmds> legReader(legSubscriber, legtopicsub,legReadQos);
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
    dds::sub::DataReader<motorcmds> armReader(armSubscriber, armtopicsub,armReadQos);
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
    dds::pub::qos::DataWriterQos legwriterQos(legtopicpubQos);  // datawriter的qos应当继承自topic的qos
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
    dds::pub::qos::DataWriterQos armwriterQos(armtopicpubQos);  // datawriter的qos应当继承自topic的qos
    dds::pub::DataWriter<motorstates> armWriter(armPublisher, armtopicpub, armwriterQos);
    std::cout << "=== [arm publisher] get ready! " << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////

    motorstates armStates;
    armStates.level(1); // 设置为上肢
    armStates.states().resize(14);
    //初始化
    for (int i = 0; i < 14; ++i) {
        auto& state = armStates.states()[i];

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
        auto& state = legStates.states()[i];

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

    while (!quit) {
        /////////////////////////////////////////////////////////////////////////////////////////////
        //读取leg订阅的消息 ---------------------------------------------------------------------------
        samples = legReader.take();
        if (samples.length() > 0) {
            dds::sub::LoanedSamples<motorcmds>::const_iterator sample_iter;
            for (sample_iter = samples.begin();sample_iter < samples.end();++sample_iter) {
                const motorcmds & legcmds = sample_iter->data();
                const dds::sub::SampleInfo& info = sample_iter->info();
                if (info.valid()) {
                    std::cout << "legMotorcmds" << (int)legcmds.level() << "   pos:" << legcmds.cmds()[0].pos() << std::endl;
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
            for (sample_iter = samples.begin();sample_iter < samples.end();++sample_iter) {
                const motorcmds & armcmds = sample_iter->data();
                const dds::sub::SampleInfo& info = sample_iter->info();
                if (info.valid()) {
                    std::cout << "legMotorcmds" << (int)armcmds.level() << "   pos:" << armcmds.cmds()[0].pos() << std::endl;
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
        for (int i = 0; i < 14; ++i) {
            auto& state = armStates.states()[i];

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
        for (int i = 0; i < 13; ++i) {
            auto& state = legStates.states()[i];

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

        armWriter.write(armStates);  //发布消息
        legWriter.write(legStates);  //发布消息

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return EXIT_FAILURE;
}
