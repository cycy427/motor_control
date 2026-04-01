/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:45:09
 * @LastEditTime: 2022-11-13 17:16:19
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <inttypes.h>
#include <string.h>
#include "config.h"
#include "math_ops.h"
#include "transmit.h"
#include <stdbool.h>
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h> 

#define param_get_pos 0x01
#define param_get_spd 0x02
#define param_get_cur 0x03
#define param_get_pwr 0x04
#define param_get_acc 0x05
#define param_get_lkgKP 0x06
#define param_get_spdKI 0x07
#define param_get_fdbKP 0x08
#define param_get_fdbKD 0x09

#define comm_ack 0x00
#define comm_auto 0x01

//以下的这些参数针对不同型号的电机这个范围是不一样的，需要根据实际情况设置，在最新版的技术手册中有说明
//下面这些不同电机也是一样的
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define POS_MIN (-12.5f)
#define POS_MAX 12.5f
#define SPD_MIN (-18.0f)
#define SPD_MAX 18.0f
//下面的KD不同YKS电机不一样
//EC-A4310-P2-36  EC-A6408-P2-25  EC-A8112-P1-18
#define KD1_MIN 0.0f
#define KD1_MAX 50.0f
//EC-A10020-P1-12/6  EC-A10020-P2-24  EC-A13720-P1-11.4  EC-A13715-P1-12.67
#define KD2_MIN 0.0f
#define KD2_MAX 50.0f

//EC-A4310-P2-36
#define KT4310 1.40f
#define T4310_MIN (-30.0f)
#define I4310_MIN (-30.0f)
#define T4310_MAX 30.0f
#define I4310_MAX 30.0f
//EC-A6408-P2-25
#define KT6408 2.35f
#define T6408_MIN (-60.0f)
#define I6408_MIN (-60.0f)
#define T6408_MAX 60.0f
#define I6408_MAX 60.0f
//EC-A8112-P1-18
#define KT8112 2.10f
#define T8112_MIN (-90.0f)
#define I8112_MIN (-60.0f)
#define T8112_MAX 90.0f
#define I8112_MAX 60.0f
//EC-A10020-P1-12/6
#define KT10020_1 2.50f
#define T10020_1_MIN (-150.0f)
#define I10020_1_MIN (-70.0f)
#define T10020_1_MAX 150.0f
#define I10020_1_MAX 70.0f
//EC-A10020-P2-24
#define KT10020_2 2.60f
#define T10020_2_MIN (-300.0f)
#define I10020_2_MIN (-140.0f)
#define T10020_2_MAX 300.0f
#define I10020_2_MAX 140.0f
//EC A13715-P1-12.67
#define KT13715 2.50f
#define T13715_MIN (-320.0f)
#define I13715_MIN (-220.0f)
#define T13715_MAX 320.0f
#define I13715_MAX 220.0f
//EC-A13720-P1-11.4
#define KT13720 2.50f
#define T13720_MIN (-400.0f)
#define I13720_MIN (-220.0f)
#define T13720_MAX 400.0f
#define I13720_MAX 220.0f
//Ti5电机
//CRA-RI30-40-PRO-101
#define GEAR_30_40_PRO 101//减速比
#define T_30_40_PRO 2.8f//2000rpm/（减速比）时额定扭矩
#define V_30_40_PRO (45.0f/0.6f)//额定转速（带1/2额定扭矩）单位：弧度/秒
#define I_30_40_PRO 1000//额定电流，单位：毫安
#define TC_30_40_PRO 0.024f//扭矩常数：N*m/A
//CRA-RI40-52-PRO-101
#define GEAR_40_52_PRO 101
#define T_40_52_PRO 6.5f
#define V_40_52_PRO (40.0f/0.6f)
#define I_40_52_PRO 2000
#define TC_40_52_PRO 0.05f
//CRA_RI50_60_PRO_101
#define GEAR_50_60_PRO 101
#define T_50_60_PRO 9.6f
#define V_50_60_PRO (37.0f/0.6f)
#define I_50_60_PRO 3600
#define TC_50_60_PRO 0.089f
//CRA_RI60_70_PRO_101
#define GEAR_60_70_PRO 101
#define T_60_70_PRO 30.0f
#define V_60_70_PRO (24.0f/0.6f)
#define I_60_70_PRO 5000
#define TC_60_70_PRO 0.096f

//CRA-RI70-90-PRO-101
#define GEAR_70_PRO 101
#define T_70_PRO 50.0f
#define V_70_PRO (25.0f/0.6f)
#define I_70_PRO 6100
#define TC_70_PRO 0.118f
//CRA-RI60-60-PRO-S-101
#define GEAR_60_PRO_S 101
#define T_60_PRO_S 9.6f
#define V_60_PRO_S (37.0f/0.6f)
#define I_60_PRO_S 3600
#define TC_60_PRO_S 0.089f
//CRA-RI70-70-PRO-S-101
#define GEAR_70_PRO_S 101
#define T_70_PRO_S 30.0f
#define V_70_PRO_S (24.0f/0.6f)
#define I_70_PRO_S 5000
#define TC_70_PRO_S 0.096f

enum YKS_MOTOR_TYPE {
    A4310 = 0,
    A6408 = 1,
    A8112 = 2,

    A10020_1 = 3,
    A10020_2 = 4,
    A13715 = 5,
    A13720 = 6
}; //YKS只有7种类型的电机
enum TI5_MOTOR_TYPE {
    CRA_RI30_40_PRO_101 = 0,
    CRA_RI40_52_PRO_101 = 1,
    CRA_RI50_60_PRO_101 = 2,
    CRA_RI60_70_PRO_101 = 3,
    CRA_RI70_90_PRO_101 = 4,
    CRA_RI80_70_PRO_S_101 = 5,
    CRA_RI90_70_PRO_S_101 = 6
}; //Ti5电机
typedef struct {
    float KD_MIN[7]; //YKS只有7种类型的电机
    float KD_MAX[7];
    float T_MIN[7];
    float T_MAX[7];
    float I_MIN[7];
    float I_MAX[7];
    float KT[7];
} YKS_MOTOR_RANGE;

// 父结构体，包含子结构体数组和其他独立参数
typedef struct {
    float GEAR_RATIO[7]; // 减速比数组
    float V_MAX[7]; // 最大速度数组
    float I_MAX[7];
    float T_MAX[7];
    float TC[7]; // 扭矩常数数组
} TI5_MOTOR_RANGE;

typedef struct {
    uint16_t motor_id;
    uint8_t INS_code; // instruction code.
    uint8_t motor_fbd; // motor CAN communication feedback.
} MotorCommFbd;

typedef struct {
    uint16_t angle_actual_int;
    uint16_t angle_desired_int;
    int16_t speed_actual_int;
    int16_t speed_desired_int;
    int16_t current_actual_int;
    int16_t current_desired_int;
    float speed_actual_rad;
    float speed_desired_rad;
    float angle_actual_rad;
    float angle_desired_rad;
    uint16_t motor_id;
    uint8_t temperature;
    uint8_t mos_temperature;
    uint8_t error;
    float angle_actual_float;
    float speed_actual_float;
    float current_actual_float;
    float angle_desired_float;
    float speed_desired_float;
    float current_desired_float;
    float power;
    uint16_t acceleration;
    uint16_t linkage_KP;
    uint16_t speed_KI;
    uint16_t feedback_KP;
    uint16_t feedback_KD;
} OD_Motor_Msg;

typedef struct {
    float angle_float[3];
    float gyro_float[3];
    float accel_float[3];
    float mag_float[3];
    float quat_float[4];
} IMU_Msg;

//-------------------------------------
// 电机类型枚举（提高判断效率）
//-------------------------------------
typedef enum {
    MOTOR_YKS, // YKS 电机
    MOTOR_TI5, // Ti5电机
    MOTOR_EYOU, // EYOU电机
} MotorType;

//-------------------------------------
// 单个电机描述结构体
//-------------------------------------
typedef enum {
    Mode_Null,
    Mode_POS_SPD,
    Mode_POS_TIME,
    Mode_SPD,
    Mode_CUR,
} EYOUMode;

//-------------------------------------
// 单个电机描述结构体
//-------------------------------------
typedef struct {
    MotorType type; // 电机类型（枚举）
    int motor_id; // 在从站内的本地编号（1~6）
    int global_id; // 全局唯一ID（可选，用于跨从站管理）
} Motor;


//-------------------------------------
// 从站（Slave）结构体
//-------------------------------------
#define MAX_MOTORS_PER_SLAVE 6

typedef struct {
    int slave_id; // SLAVE ID (1-5)
    int motor_count; // 当前从站连接的电机数量
    Motor motors[MAX_MOTORS_PER_SLAVE]; // 电机列表
} Slave;

//-------------------------------------
// EYOU电机初始化结构体
//-------------------------------------

typedef struct {
    bool ISENABLE; 
    bool ISSETMODE; 
    bool ISSETSPD;
} ISEYOU_Init;

// extern ISEYOU_Init eyou_init_status[MAX_MOTORS_PER_SLAVE] = {0}; // 每个从站每个电机的初始化状态

extern Slave g_slaves[SLAVE_NUMBER];


extern OD_Motor_Msg rv_motor_msg[6]; //每块板子上面只能连接6个电机
extern IMU_Msg imu_msg;
extern uint16_t motor_id_check;

void MotorIDReset(EtherCAT_Msg *TxMessage);

void MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new);

void MotorSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t cmd);

void MotorCommModeReading(EtherCAT_Msg *TxMessage, uint16_t motor_id);

void MotorIDReading(EtherCAT_Msg *TxMessage);

void send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, float kp, float kd,
                         float pos,
                         float spd, float cur);

void
set_motor_position(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, float pos, uint16_t spd,
                   uint16_t cur,
                   uint8_t ack_status);

void set_motor_speed(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, float spd, uint16_t cur,
                     uint8_t ack_status);

void set_motor_cur_tor(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, int16_t cur_tor,
                       uint8_t ctrl_status, uint8_t ack_status);

void set_motor_acceleration(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, uint16_t acc,
                            uint8_t ack_status);

void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, uint16_t linkage,
                               uint16_t speedKI, uint8_t ack_status);

void
set_motor_feedbackKP(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, uint16_t fdbKP,
                     uint8_t ack_status);

void get_motor_parameter(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint16_t motor_id, uint8_t param_cmd);

void RV_can_data_repack(const EtherCAT_Msg *RxMessage, uint8_t comm_mode, const Slave *slave,
                        uint8_t *motor_ack_status);

void RV_can_imu_data_repack(EtherCAT_Msg *RxMessage);

void Rv_Message_Print(uint8_t ack_status);

void set_ti5_current(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float cur);

void set_ti5_speed(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float spd);

void set_ti5_position(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float pos);

void set_ti5_stop(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, int times);

void set_eyou_enable(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, bool on);

void set_eyou_mode(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, uint8_t mode);  

void notify_eyou_enabled(int idx);

void notify_eyou_mode_set(int idx);

void set_eyou_current(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float cur);   

void set_eyou_speed(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float spd);

void set_eyou_position(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float pos); 

void set_eyou_acceleration(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float acc);

void set_eyou_deceleration(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float dec);

void set_eyou_stop(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id);

#endif
