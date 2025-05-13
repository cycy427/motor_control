/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 11:17:58
 * @LastEditTime: 2022-11-13 17:16:18
 */
#ifndef TRANSMIT_H
#define TRANSMIT_H

#define SLAVE_NUMBER 5 //可接最大从机数
#define YKS_MOTOR_NUMBER 13 //接入的YKS电机数
#define TI5_MOTOR_NUMBER 14 //接入的Ti5电机数
#define TOTAL_MOTOR_NUMBER 27 //接入的总电机数

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "ethercat.h"
#include "sys/time.h"


#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    int mode;
    double pos_, vel_, tau_; //当前的位置rad、速度rad/s、力矩N.m
    double pos_des_, vel_des_, kp_, kd_, ff_; //期望的位置、速度、比例、积分、力矩N.m
    // mfn -> 新加
    int error_;
    double temperature_, mos_temperature_;
} YKSMotorData;

extern YKSMotorData motorDate_recv[TOTAL_MOTOR_NUMBER];

typedef struct {
    double angle_float[3];
    double gyro_float[3];
    double accel_float[3];
    double mag_float[3];
    double quat_float[4];
} YKS_IMUData;

extern YKS_IMUData imuData_recv;


int CAT_Init(const char *device_name);

void EtherCAT_Transmit();

void EtherCAT_Init(const char *if_name);

void EtherCAT_Run();

void EtherCAT_Command_Set();

void startRun();

void EtherCAT_Send_Command(const YKSMotorData *mot_data);

void EtherCAT_Get_State(uint8_t slave, const uint8_t *motor_ack_status);

void User_Get_Motor_Data(YKSMotorData *mot_data);
#ifdef __cplusplus
};
#endif

#endif // PROJECT_RT_ETHERCAT_H
