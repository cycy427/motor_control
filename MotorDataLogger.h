//
// Created by luoyukun on 25-5-21.
//
#ifndef MOTORDATALOGGER_H
#define MOTORDATALOGGER_H

#include <cstdio>
#include <thread>
#include <atomic>
#include <mutex>
#include "app/command.h"

#include <chrono>
#include <string>
#include <ncurses.h>
#include <cstring>

#include <fstream>
#include <sstream>
#include <filesystem>
#include <ctime>
constexpr int NUM_MOTOR = TOTAL_MOTOR_NUMBER;


class MotorDataLogger {
public:
    explicit MotorDataLogger();

    void print_log(const YKSMotorData *data);//保存日志

    ~MotorDataLogger();

    std::atomic<bool> stop_{};

private:
    static std::string getCurrentTimestamp();

    void logMotorDataToFile();

    std::shared_ptr<std::thread> loggingThread;

    bool isLogging = false;

    YKSMotorData motor_data_[NUM_MOTOR]{}; //私有的电机结构体数组
    mutable std::mutex mutex_; //用于电机数据读取与写入的互斥锁


};

#endif // MOTORDATALOGGER_H
