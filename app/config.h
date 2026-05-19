/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:21:54
 * @LastEditTime: 2022-11-13 17:00:10
 */
#pragma once

#include <inttypes.h>
#include <string.h>

#define CAN_CHANNEL_NUMBER 3
#define MOTOR_PER_CAN_CHANNEL 8
#define ACTIVE_PDO_SLOT_NUMBER (CAN_CHANNEL_NUMBER * MOTOR_PER_CAN_CHANNEL)
// The new EtherCAT-CANFD slave reserves 40 PDO frame slots. The current
// firmware uses the first 24 slots as 3 CANFD buses x 8 frames.
#define MOTOR_NUMBER 40

//********************************************//
//***********EtherCAT Message*****************//
//********************************************//

#pragma pack(push, 1)

typedef struct Motor_Msg
{
    uint32_t id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
} Motor_Msg;

typedef struct
{
    uint8_t motor_num;
    uint8_t can_ide;
    Motor_Msg motor[MOTOR_NUMBER];

} EtherCAT_Msg;

#pragma pack(pop)
