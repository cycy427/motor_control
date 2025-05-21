//
// Created by nubot on 25-5-21.
//

#include "MotorDataLogger.h"

MotorDataLogger::MotorDataLogger() : stop_(false) {
    if (!isLogging) {
        isLogging = true;
    }
    loggingThread = std::make_shared<std::thread>(&MotorDataLogger::logMotorDataToFile, this);
}

MotorDataLogger::~MotorDataLogger() {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    if (loggingThread->joinable()) {
        loggingThread->join();
    }
}

void MotorDataLogger::print_log(const YKSMotorData *data) {
    // 检查输入数据指针是否为空
    if (data == nullptr) {
        throw std::invalid_argument("Input data pointer cannot be null");
    }
    std::lock_guard lock(mutex_);

    for (int i = 0; i < NUM_MOTOR; ++i) {
        motor_data_[i].mode = data[i].mode;
        motor_data_[i].pos_des_ = data[i].pos_des_;
        motor_data_[i].vel_des_ = data[i].vel_des_;
        motor_data_[i].ff_ = data[i].ff_;
        motor_data_[i].pos_ = data[i].pos_;
        motor_data_[i].vel_ = data[i].vel_;
        motor_data_[i].tau_ = data[i].tau_;
        motor_data_[i].error_ = data[i].error_;
        motor_data_[i].temperature_ = data[i].temperature_;
        motor_data_[i].mos_temperature_ = data[i].mos_temperature_;
    }
}


std::string MotorDataLogger::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    // 使用 localtime_s 或 localtime_r（跨平台注意）
    struct tm time_info;
#ifdef _WIN32
    localtime_s(&time_info, &now_time_t);
#else
    localtime_r(&now_time_t, &time_info);
#endif

    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &time_info);

    // 拼接毫秒部分
    char timestamp[84];
    snprintf(timestamp, sizeof(timestamp), "%s.%03lld", buffer, (long long)ms.count());

    return std::string(timestamp);
}

// 记录电机数据到 CSV 文件
void MotorDataLogger::logMotorDataToFile() {
    // 创建日志目录（如果不存在）
    std::string logDir = "log";
    if (!std::filesystem::exists(logDir)) {
        std::filesystem::create_directory(logDir);
    }

    // 构建文件名（基于时间戳）
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    char fileName[100];
    std::strftime(fileName, sizeof(fileName), "motor_data_%Y%m%d_%H%M%S.csv", std::localtime(&now_time));
    std::string filePath = logDir + "/" + std::string(fileName);

    // 打开文件并写入表头
    std::ofstream outFile(filePath, std::ios_base::app); // 接续保存
    outFile << "timestamp,motor_index,tau,error,temperature,mos_temperature\n";

    while (isLogging) {
        std::string timestamp = getCurrentTimestamp();

        for (int i = 0; i < NUM_MOTOR; ++i) {
            std::lock_guard<std::mutex> lock(mutex_);
            outFile << timestamp << ","
                    << i << ","
                    << motor_data_[i].tau_ << ","
                    << motor_data_[i].error_ << ","
                    << motor_data_[i].temperature_ << ","
                    << motor_data_[i].mos_temperature_ << "\n";
        }

        outFile.flush(); // 刷新缓冲区，确保数据写入磁盘

        // 控制频率为10Hz（100ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    outFile.close();
}
