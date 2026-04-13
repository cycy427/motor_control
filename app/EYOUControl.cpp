#include "EYOUControl.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>

extern std::mutex message_mutex;
extern EtherCAT_Msg Tx_Message[SLAVE_NUMBER];

namespace {

constexpr uint8_t kEyouMinChannel = 1;
constexpr uint8_t kEyouMaxChannel = 6;


/**
 * @brief 确认通道是否合法，在1-6之间
 * @param[in] data_channel: 1-6，表示要控制的电机在从站内的通道号
 */
bool isValidEyouChannel(uint8_t data_channel) {
    return data_channel >= kEyouMinChannel && data_channel <= kEyouMaxChannel;
}

/**
 * @brief 配置除data段以外的协议部分，其中data[0]固定为0x01,表示写命令；其中data[1]是要写入的地址
 * @param[in] tx_message: 指向要发送的 EtherCAT 消息结构体指针
 * @param[in] data_channel: 1-6，表示要控制的电机在从站内的通道号
 * @param[in] motor_id: 电机实际ID，等于global_id + 1
 * @param[in] sub_cmd: 要写入的地址
 */
void prepareEyouCommand(EtherCAT_Msg *tx_message, uint8_t data_channel, uint32_t motor_id, uint8_t sub_cmd) {
    if (tx_message == nullptr || !isValidEyouChannel(data_channel)) {
        return;
    }

    Motor_Msg &motor_slot = tx_message->motor[data_channel - 1];
    tx_message->can_ide = 0;
    motor_slot.rtr = 0;
    motor_slot.id = motor_id;
    motor_slot.dlc = 8;
    std::memset(motor_slot.data, 0, sizeof(motor_slot.data));
    motor_slot.data[0] = 0x01;
    motor_slot.data[1] = sub_cmd;
}

/**
 * @brief 写数据帧里的data段，是协议里的数据部分
 * @param[in] tx_message: 指向要发送的 EtherCAT 消息结构体指针
 * @param[in] data_channel: 1-6，表示要控制的电机在从站内的通道号
 * @param[in] value: 要写入的值，会被转换成4字节大端整数写入data[2]~data[5]
 */
void writeEyouPayloadInt32(EtherCAT_Msg *tx_message, uint8_t data_channel, int32_t value) {
    if (tx_message == nullptr || !isValidEyouChannel(data_channel)) {
        return;
    }

    Motor_Msg &motor_slot = tx_message->motor[data_channel - 1];
    motor_slot.data[2] = static_cast<uint8_t>(value >> 24);
    motor_slot.data[3] = static_cast<uint8_t>(value >> 16);
    motor_slot.data[4] = static_cast<uint8_t>(value >> 8);
    motor_slot.data[5] = static_cast<uint8_t>(value);
}

/**
 * @brief 将弧度值转换成EYOU电机原始数据(脉冲个数)，65536个脉冲对应2π弧度，也就是电机转一圈
 * @param[in] value: 要转换的弧度值
 * @return 转换后的整数值
 */
int32_t radiansToEyouRaw(float value) {
    return static_cast<int32_t>(value / static_cast<float>(2.0 * M_PI) * 65536.0f);
}

/**
 * @brief 清除tx_message里的配置内容，避免旧配置干扰
 * @param[in] tx_message: 指向要发送的 EtherCAT 消息结构体指针
 * @param[in] data_channel: 1-6，表示要控制的电机在从站内的通道号
 */
void clear_tx_motor_slot(EtherCAT_Msg *tx_message, uint8_t data_channel) {
    if (tx_message == nullptr || data_channel < 1 || data_channel > 6) {
        return;
    }

    Motor_Msg &motor_slot = tx_message->motor[data_channel - 1];
    motor_slot.id = 0;
    motor_slot.rtr = 0;
    motor_slot.dlc = 0;
    std::memset(motor_slot.data, 0, sizeof(motor_slot.data));
}

class EyouRuntimeState {
public:
    /**
     * @brief 通知电机已使能，可以进入后续的模式设置和控制指令发送流程
     * @param[in] idx: 电机的global_id
     */
    void notifyEnabled(int idx) {
        if (!isValidGlobalId(idx)) {
            return;
        }
        enabled_[idx].store(true, std::memory_order_release);
    }

    /**
     * @brief 通知电机已完成了模式配置，可以进入后续的控制指令发送流程
     * @param[in] idx: 电机的global_id
     */
    void notifyModeSet(int idx) {
        if (!isValidGlobalId(idx)) {
            return;
        }
        mode_set_[idx].store(true, std::memory_order_release);
    }

    /**
     * @brief 通知电机已完成了位置速度配置，可以进入后续的控制指令发送流程，这里是位置轮廓模式要运行的必要配置之一
     * @param[in] idx: 电机的global_id
     */
    void notifyProfileSpeedSet(int idx) {
        if (!isValidGlobalId(idx)) {
            return;
        }
        profile_speed_set_[idx].store(true, std::memory_order_release);
    }

    /**
     * @brief 进行模式合法判定，非法模式报错，同时初始化配置，这里就是初始化配置的位置，但是进来一次只进行一次初始化配置
     * @param[in] motor: 电机信息结构体指针
     * @param[in] slave_idx: 从站索引
     * @param[in] requested_mode: 请求的模式值，0表示无效模式，1表示位置速度模式，2表示电流模式，3表示速度模式
     * @param[in] mot_data: 包含电机状态和期望状态信息的数组指针
     */
    void handleInitOnly(const Motor *motor, int slave_idx, int requested_mode, const YKSMotorData *mot_data) {
        if (!shouldHandleMotor(motor)) {
            return;
        }
        
        // 这里是不让mode0成为一个非法模式的初始化命令
        if (requested_mode == 0) {
            clearInvalidModeReport(motor->global_id);
            return;
        }

        const uint8_t target_mode = getTargetMode(requested_mode);
        if (!isValidTargetMode(target_mode)) {
            reportInvalidMode(motor, requested_mode);
            return;
        }

        clearInvalidModeReport(motor->global_id);
        if (!queueInitStep(motor, slave_idx, target_mode)) {
            std::lock_guard<std::mutex> lock(message_mutex);
            clear_tx_motor_slot(&Tx_Message[slave_idx], motor->motor_id);
        }
    }

    /**
     * @brief 进行一些合法判定，发送模式控制指令，这里是模式控制的入口
     * @param[in] motor: 电机信息结构体指针
     * @param[in] slave_idx: 从站索引
     * @param[in] requested_mode: 请求的模式值，0表示无效模式，1表示位置速度模式，2表示电流模式，3表示速度模式
     * @param[in] mot_data: 包含电机状态和期望状态信息的数组指针
     */
    void handleRuntimeCommand(const Motor *motor, int slave_idx, int index, const YKSMotorData *mot_data) {
        if (!shouldHandleMotor(motor)) {
            return;
        }

        if (mot_data[index].mode == 0) {
            clearInvalidModeReport(motor->global_id);
            std::lock_guard<std::mutex> lock(message_mutex);
            clear_tx_motor_slot(&Tx_Message[slave_idx], motor->motor_id);
            return;
        }

        const uint8_t target_mode = getTargetMode(mot_data[index].mode);
        if (!isValidTargetMode(target_mode)) {
            reportInvalidMode(motor, mot_data[index].mode);
            return;
        }

        if (shouldBlockRuntimeModeSwitch(motor->global_id, target_mode)) {
            if (!canSwitchModeSafely(mot_data, index)) {
                reportUnsafeModeSwitch(motor, target_mode, mot_data, index);
                std::lock_guard<std::mutex> lock(message_mutex);
                clear_tx_motor_slot(&Tx_Message[slave_idx], motor->motor_id);
                return;
            }
            clearUnsafeModeSwitchReport(motor->global_id);
        } else {
            clearUnsafeModeSwitchReport(motor->global_id);
        }

        clearInvalidModeReport(motor->global_id);
        if (queueInitStep(motor, slave_idx, target_mode)) {
            return;
        }

        std::lock_guard<std::mutex> lock(message_mutex);
        if (target_mode == Mode_POS_SPD) {
            if (!shouldSendPositionCommand(motor->global_id, mot_data[index].pos_des_)) {
                clear_tx_motor_slot(&Tx_Message[slave_idx], motor->motor_id);
                return;
            }
            set_eyou_position(&Tx_Message[slave_idx], motor->motor_id, motor->motor_id, mot_data[index].pos_des_);
            markPositionCommandSent(motor->global_id, mot_data[index].pos_des_);
        } else if (target_mode == Mode_CUR) {
            set_eyou_current(&Tx_Message[slave_idx], motor->motor_id, motor->motor_id, mot_data[index].ff_);
        } else if (target_mode == Mode_SPD) {
            set_eyou_speed(&Tx_Message[slave_idx], motor->motor_id, motor->motor_id, mot_data[index].vel_des_);
        }
    }

private:
    static constexpr float kDefaultProfilePositionSpeed = static_cast<float>(M_PI) / 2.0f;
    static constexpr auto kPositionCommandInterval = std::chrono::microseconds(66667);
    static constexpr double kPositionCommandEpsilon = 1e-4;
    static constexpr float kRuntimeModeSwitchVelocityThreshold = 0.05f;                 // 单位是rad/s，允许电机切换模式的门槛速度
    static constexpr float kRuntimeModeSwitchTorqueThreshold = 0.5f;                    // 单位是N.m，允许电机切换模式的门槛力矩

    /**
     * @brief 判断global_id是否合法，应该在0到TOTAL_MOTOR_NUMBER-1之间
     * @param[in] global_id: 电机的global_id
     */
    static bool isValidGlobalId(int global_id) {
        return global_id >= 0 && global_id < TOTAL_MOTOR_NUMBER;
    }

    /**
     * @brief 判断是否需要处理这个电机的指令，条件是电机类型是EYOU且global_id合法
     * @param[in] motor: 要判断的电机信息结构体指针
     */
    static bool shouldHandleMotor(const Motor *motor) {
        return motor->type == MOTOR_EYOU && isValidGlobalId(motor->global_id);
    }

    /**
     * @brief 根据请求的模式值返回对应的EYOU模式编码，1对应位置速度模式，2对应电流模式，3对应速度模式，其他值返回0表示无效模式
     * @param[in] requested_mode: 请求的模式值
     */
    static uint8_t getTargetMode(int requested_mode) {
        if (requested_mode == 1) {
            return Mode_POS_SPD;
        }
        else if (requested_mode == 2) {
            return Mode_CUR;
        }
        else if (requested_mode == 3) {
            return Mode_SPD;
        }
        return Mode_Null;
    }

    /**
     * @brief 判定目标模式是否合法，EYOU电机只支持位置速度模式、电流模式和速度模式
     * @param[in] target_mode: 目标模式编码
     */
    static bool isValidTargetMode(uint8_t target_mode) {
        return target_mode == Mode_POS_SPD || target_mode == Mode_CUR || target_mode == Mode_SPD;
    }

    /**
     * @brief 获取位置轮廓模式的固定轮廓速度。进入位置模式时只配置一次，后续位置控制只发送位置命令。
     * @return 固定的轮廓速度值
     */
    static float getProfilePositionSpeed() {
        return kDefaultProfilePositionSpeed;
    }

    void resetPositionCommandState(int global_id) {
        if (!isValidGlobalId(global_id)) {
            return;
        }

        last_position_command_time_[global_id] = std::chrono::steady_clock::time_point::min();
        last_position_command_target_[global_id] = 0.0;
        has_position_command_target_[global_id] = false;
    }

    bool shouldSendPositionCommand(int global_id, double target_position) const {
        if (!isValidGlobalId(global_id)) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!has_position_command_target_[global_id]) {
            return true;
        }

        const auto elapsed = now - last_position_command_time_[global_id];
        if (elapsed < kPositionCommandInterval) {
            return false;
        }

        return std::fabs(target_position - last_position_command_target_[global_id]) > kPositionCommandEpsilon;
    }

    void markPositionCommandSent(int global_id, double target_position) {
        if (!isValidGlobalId(global_id)) {
            return;
        }

        last_position_command_time_[global_id] = std::chrono::steady_clock::now();
        last_position_command_target_[global_id] = target_position;
        has_position_command_target_[global_id] = true;
    }

    /**
     * @brief 清除之前的无效模式报告状态
     * @param[in] global_id: 电机的global_id
     */
    void clearInvalidModeReport(int global_id) {
        if (!isValidGlobalId(global_id)) {
            return;
        }
        invalid_mode_reported_[global_id] = false;
    }

    /**
     * @brief 判断电机是否处于活动模式状态
     * @param[in] global_id: 电机的global_id
     */
    bool hasActiveModeState(int global_id) const {
        if (!isValidGlobalId(global_id)) {
            return false;
        }

        return active_mode_[global_id] != Mode_Null ||
               enabled_[global_id].load(std::memory_order_acquire) ||
               mode_set_[global_id].load(std::memory_order_acquire);
    }

    /**
     * @brief 判断在运行时模式切换请求是否应该被拒绝，条件是当前已经有一个有效的目标模式了，并且和请求的目标模式不一样，
     *        这种情况说明用户在试图切换到另一个模式，但是电机还没有完全停止或者没有完全进入空闲状态，如果直接切换可能会有风险，所以先拒绝切换请求，等电机状态满足安全切换的条件了再允许切换
     * @param[in] global_id: 电机的global_id
     * @param[in] target_mode: 请求切换的目标模式编码
     */
    bool shouldBlockRuntimeModeSwitch(int global_id, uint8_t target_mode) const {
        return hasActiveModeState(global_id) &&
               active_mode_[global_id] != Mode_Null &&
               active_mode_[global_id] != target_mode;
    }

    /**
     * @brief 判断电机当前的状态是否满足安全切换模式的条件，条件是电机当前的速度和力矩都要非常小，说明电机基本处于静止状态了，这时候切换模式风险较小
     * @param[in] mot_data: 包含电机状态和期望状态信息的数组指针
     * @param[in] index: 电机在从站内的通道索引，0-5对应通道1-6
     */
    static bool canSwitchModeSafely(const YKSMotorData *mot_data, int index) {
        return std::fabs(static_cast<float>(mot_data[index].vel_)) <= kRuntimeModeSwitchVelocityThreshold &&
               std::fabs(static_cast<float>(mot_data[index].tau_)) <= kRuntimeModeSwitchTorqueThreshold;
    }

    void clearUnsafeModeSwitchReport(int global_id) {
        if (!isValidGlobalId(global_id)) {
            return;
        }

        unsafe_mode_switch_reported_[global_id] = false;
    }

    void reportUnsafeModeSwitch(const Motor *motor, uint8_t target_mode, const YKSMotorData *mot_data, int index) {
        if (!isValidGlobalId(motor->global_id)) {
            return;
        }

        if (!unsafe_mode_switch_reported_[motor->global_id] || last_rejected_mode_[motor->global_id] != target_mode) {
            printf("[EYOU Warn] Reject runtime mode switch for global_id=%d slave_motor=%d: current_mode=%u target_mode=%u vel=%.4f rad/s tau=%.4f. Wait until the motor is nearly stationary before switching modes.\n",
                   motor->global_id,
                   motor->motor_id,
                   static_cast<unsigned>(active_mode_[motor->global_id]),
                   static_cast<unsigned>(target_mode),
                   mot_data[index].vel_,
                   mot_data[index].tau_);
            unsafe_mode_switch_reported_[motor->global_id] = true;
            last_rejected_mode_[motor->global_id] = target_mode;
        }
    }

    /**
     * @brief 报告一个无效的模式请求，只有当之前没有报告过或者这次请求的模式和上次报告的模式不一样时才打印日志，避免日志刷屏
     * @param[in] motor: 发送无效模式请求的电机信息结构体指针
     * @param[in] requested_mode: 请求的无效模式值
     */
    void reportInvalidMode(const Motor *motor, int requested_mode) {
        if (!isValidGlobalId(motor->global_id)) {
            return;
        }

        if (!invalid_mode_reported_[motor->global_id] || last_invalid_mode_[motor->global_id] != requested_mode) {
            printf("[EYOU Error] Invalid control mode=%d for global_id=%d slave_motor=%d. Supported modes: 1(position), 2(current), 3(speed).\n",
                   requested_mode, motor->global_id, motor->motor_id);
            invalid_mode_reported_[motor->global_id] = true;
            last_invalid_mode_[motor->global_id] = requested_mode;
        }
    }

    void resetInitState(int global_id, uint8_t target_mode) {
        enabled_[global_id].store(false, std::memory_order_release);
        mode_set_[global_id].store(false, std::memory_order_release);
        profile_speed_set_[global_id].store(target_mode != Mode_POS_SPD, std::memory_order_release);
        active_mode_[global_id] = target_mode;
        resetPositionCommandState(global_id);
    }

    /**
     * @brief 根据当前的初始化状态和请求的模式判断是否需要继续执行初始化步骤，按照使能->模式设置->位置速度配置的顺序依次进行，并且每一步只发送一次命令，直到都完成了才返回false表示不需要继续初始化了
     * @param[in] motor: 要控制的电机信息结构体指针
     * @param[in] slave_idx: 电机所在的从站索引
     * @param[in] target_mode: 目标模式编码
     * @return 如果需要继续执行初始化步骤返回true，否则返回false
     */
    bool queueInitStep(const Motor *motor, int slave_idx, uint8_t target_mode) {
        if (!shouldHandleMotor(motor)) {
            return false;
        }

        if (active_mode_[motor->global_id] != target_mode) {
            resetInitState(motor->global_id, target_mode);
        }

        if (!enabled_[motor->global_id].load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(message_mutex);
            set_eyou_enable(&Tx_Message[slave_idx], motor->motor_id, motor->motor_id, true);
            return true;
        }

        if (!mode_set_[motor->global_id].load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(message_mutex);
            set_eyou_mode(&Tx_Message[slave_idx], motor->motor_id, motor->motor_id, target_mode);
            return true;
        }

        if (target_mode == Mode_POS_SPD && !profile_speed_set_[motor->global_id].load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(message_mutex);
            set_eyou_speed(&Tx_Message[slave_idx], motor->motor_id, motor->motor_id,
                           getProfilePositionSpeed());
            return true;
        }

        return false;
    }

    std::atomic<bool> enabled_[TOTAL_MOTOR_NUMBER]{};               // 每台电机是否已经收到使能成功反馈
    std::atomic<bool> mode_set_[TOTAL_MOTOR_NUMBER]{};              // 每台电机是否已经完成模式配置
    std::atomic<bool> profile_speed_set_[TOTAL_MOTOR_NUMBER]{};     // 位置轮廓模式下，固定轮廓速度是否已经配置完成
    uint8_t active_mode_[TOTAL_MOTOR_NUMBER]{};                     // 每台电机当前记录的目标工作模式
    bool invalid_mode_reported_[TOTAL_MOTOR_NUMBER]{};              // 是否已经打印过当前电机的非法模式告警
    int last_invalid_mode_[TOTAL_MOTOR_NUMBER]{};                   // 上一次触发非法模式告警的模式值，用于去重
    bool unsafe_mode_switch_reported_[TOTAL_MOTOR_NUMBER]{};        // 是否已经打印过当前电机的不安全切模告警
    int last_rejected_mode_[TOTAL_MOTOR_NUMBER]{};                  // 上一次因不安全被拒绝切换的目标模式，用于去重
    std::chrono::steady_clock::time_point last_position_command_time_[TOTAL_MOTOR_NUMBER]{}; // 上一次发送位置命令的时间戳
    double last_position_command_target_[TOTAL_MOTOR_NUMBER]{};     // 上一次发送的位置目标值，用于抑制重复下发
    bool has_position_command_target_[TOTAL_MOTOR_NUMBER]{};        // 是否已经记录过位置目标，用于首次发送判断
};

EyouRuntimeState g_eyou_runtime;

} // namespace

extern "C" void set_eyou_enable(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, bool on) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x10);
    TxMessage->motor[data_channel - 1].data[5] = on ? 1u : 0u;
}

extern "C" void set_eyou_mode(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, uint8_t mode) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x0F);
    TxMessage->motor[data_channel - 1].data[5] = mode;
}

extern "C" void set_eyou_current(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float cur) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x08);
    writeEyouPayloadInt32(TxMessage, data_channel, static_cast<int32_t>(cur));
}

extern "C" void set_eyou_speed(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float spd) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x09);
    writeEyouPayloadInt32(TxMessage, data_channel, radiansToEyouRaw(spd));
}

extern "C" void set_eyou_position(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float pos) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x0A);
    writeEyouPayloadInt32(TxMessage, data_channel, radiansToEyouRaw(pos));
}

extern "C" void set_eyou_acceleration(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float acc) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x0B);
    writeEyouPayloadInt32(TxMessage, data_channel, radiansToEyouRaw(acc));
}

extern "C" void set_eyou_deceleration(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float dec) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x0C);
    writeEyouPayloadInt32(TxMessage, data_channel, radiansToEyouRaw(dec));
}

extern "C" void set_eyou_stop(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id) {
    if (!isValidEyouChannel(data_channel)) {
        return;
    }

    prepareEyouCommand(TxMessage, data_channel, motor_id, 0x10);
}

void EyouHandleInitOnly(const Motor *motor, int slave_idx, int requested_mode, const YKSMotorData *mot_data) {
    g_eyou_runtime.handleInitOnly(motor, slave_idx, requested_mode, mot_data);
}

void EyouHandleRuntimeCommand(const Motor *motor, int slave_idx, int index, const YKSMotorData *mot_data) {
    g_eyou_runtime.handleRuntimeCommand(motor, slave_idx, index, mot_data);
}

extern "C" void notify_eyou_enabled(int idx) {
    g_eyou_runtime.notifyEnabled(idx);
}

extern "C" void notify_eyou_mode_set(int idx) {
    g_eyou_runtime.notifyModeSet(idx);
}

extern "C" void notify_eyou_profile_speed_set(int idx) {
    g_eyou_runtime.notifyProfileSpeedSet(idx);
}

