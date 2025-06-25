/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-13 19:00:55
 * @LastEditTime: 2022-11-13 17:09:03
 */


#include <cstdio>
#include "Console.hpp"
#include "command.h"

extern "C" {
#include "ethercat.h"
}
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

namespace cr = CppReadline;
using ret = cr::Console::ReturnCode;
std::atomic<bool> stop_thread(false);
volatile sig_atomic_t stop_flag = false; // 使用不带 std:: 的 sig_atomic_t

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nReceived Ctrl+C, exiting gracefully..." << std::endl;
        stop_flag = true; // 设置退出标志
    }
}
int main() {
    // std::signal(SIGINT, signal_handler);

    printf("SOEM 主站测试\n");

    EtherCAT_Init("enp3s0");

    if (ec_slavecount <= 0) {
        printf("未找到从站, 程序退出！");
        return 1;
    } else
        printf("从站数量： %d\r\n", ec_slavecount);

    startRun();

    cr::Console cli("[Command] > ");
    cli.registerCommand("help", help);
    cli.registerCommand("MotorIdGet", motorIdGet);
    cli.registerCommand("MotorIdSet", motorIdSet);
    cli.registerCommand("MotorSpeedSet", motorSpeedSet);
    cli.registerCommand("MotorPositionSet", motoPositionSet);

    int retCode;
    do {
        retCode = cli.readLine();
    } while (retCode != ret::Quit);

    runThread.join();
    return 0;
}
