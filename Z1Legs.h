//
// Created by lijinzhe on 25-3-1.
//
#ifndef YKS_SDK_Z1LEGS_H
#define YKS_SDK_Z1LEGS_H

#define PRINT_MOTOR_STATE //如果想要显示机器人的所有电机的状态，请将此宏定义开启

#include <cstdio>
#include <array>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include "app/command.h"
#include "JoyStickHandler.h"
#include "BmsHandler.h"
#include <chrono>
#include <string>
#include <ncurses.h>
#include <cstring>

extern "C" {
#include "ethercat.h"
}

constexpr int Z1_NUM_MOTOR = YKS_MOTOR_NUMBER;

enum class Mode {
    PR = 0, // Series Control for Pitch/Roll Joints
    AB = 1 // Parallel Control for A/B Joints
};

enum Z1JointIndex {
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    //下面四个待定
    LeftAnklePitch = 4,
    LeftAnkleB = 4,
    LeftAnkleRoll = 5,
    LeftAnkleA = 5,

    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,

    RightAnklePitch = 10,
    RightAnkleB = 10,
    RightAnkleRoll = 11,
    RightAnkleA = 11,

    WaistYaw = 12
};

class Z1Legs {
public:
    explicit Z1Legs();

    void setJoyStickHandler(const std::shared_ptr<JoyStickHandler> &handler);

    void setBatteryHandler(const std::shared_ptr<BmsHandler> &handler);

    void getMotorData(YKSMotorData *data) const; //获取电机的状态

    void setMotorCommand(const YKSMotorData *data); //设置电机的位置、速度、力

    void setMotorKpKd(const YKSMotorData *data); //设置电机的Kp和Kd

    ~Z1Legs();

    std::atomic<bool> stop_{};

private:
    void Control();

    void updateMotorData();

    static void PrintFrequency(int &iteration_count, std::chrono::high_resolution_clock::time_point &start_time);

    void PrintMotorState(int size) const;

    YKSMotorData Pitch_forward_kinematics(const YKSMotorData &Ankle_A_motors, const YKSMotorData &Ankle_B_motors) const;

    //通过脚踝AB电机的角度计算耦合的踝关节的俯仰角
    YKSMotorData Roll_forward_kinematics(const YKSMotorData &Ankle_A_motors, const YKSMotorData &Ankle_B_motors) const;

    //通过脚踝电机的位置计算耦合的踝关节的横滚角
    //输入目标踝关节的俯仰角横滚角计算脚踝A电机的旋转角度
    YKSMotorData AnkleA_inverse_kinematics(const YKSMotorData &pitch_joint_cmd,
                                           const YKSMotorData &roll_joint_cmd) const;

    //输入目标踝关节的俯仰角横滚角计算脚踝B电机的旋转角度
    YKSMotorData AnkleB_inverse_kinematics(const YKSMotorData &pitch_joint_cmd,
                                           const YKSMotorData &roll_joint_cmd) const;

    std::shared_ptr<std::thread> control_thread_;
    // Stiffness for all Z1 Joints
    std::array<float, Z1_NUM_MOTOR> Kp{
        150, 150, 150, 150, 15, 15, // legs
        150, 150, 150, 150, 150, 150, // legs
        150 // waist
        //        60, 40, 40,                   // waist
        //        40, 40, 40, 40, 40, 40, 40,  // arms
        //        40, 40, 40, 40, 40, 40, 40   // arms
    };
    // Damping for all Z1 Joints
    std::array<float, Z1_NUM_MOTOR> Kd{
        50, 50, 50, 50, 0.5, 0.5, // legs
        50, 50, 50, 50, 50, 50, // legs
        50 // waist
        //        1, 1, 1,              // waist
        //        1, 1, 1, 1, 1, 1, 1,  // arms
        //        1, 1, 1, 1, 1, 1, 1   // arms
    };
    double time_;
    int control_dt_; // [2ms]
    double duration_; // [3 s]
    int counter_;
    Mode mode_pr_; //启用PR模式俯仰角横滚角控制还是AB模式单独控制两个脚踝的电机  默认为PR模式
    const std::vector<double> PR_directionMotor_ = {-1, 1};
    uint8_t mode_machine_;
    YKSMotorData motor_data_[Z1_NUM_MOTOR]{}; //私有的电机结构体数组
    mutable std::mutex mutex_; //用于电机数据读取与写入的互斥锁
    bool joystick_enable_ = false; //是否在这个类当中开启了JoyStick
    std::shared_ptr<JoyStickHandler> joy_stick_handler_; // 添加 JoyStickHandler 成员变量
    bool battery_enable_ = false; //是否在这个类当中传入了电池的句柄
    std::shared_ptr<BmsHandler> battery_handler_; // 添加BatteryHandler成员变量
};

#endif //YKS_SDK_Z1LEGS_H
