#include "motor_control.h"

YKS_MOTOR_RANGE yks_motor_range = {
    .KD_MIN = {KD1_MIN, KD1_MIN, KD1_MIN, KD2_MIN, KD2_MIN, KD2_MIN, KD2_MIN},
    .KD_MAX = {KD1_MAX, KD1_MAX, KD1_MAX, KD2_MAX, KD2_MAX, KD2_MAX, KD2_MAX},
    .T_MIN = {T4310_MIN, T6408_MIN, T8112_MIN, T10020_1_MIN, T10020_2_MIN, T13715_MIN, T13720_MIN},
    .T_MAX = {T4310_MAX, T6408_MAX, T8112_MAX, T10020_1_MAX, T10020_2_MAX, T13715_MAX, T13720_MAX},
    .I_MIN = {I4310_MIN, I6408_MIN, I8112_MIN, I10020_1_MIN, I10020_2_MIN, I13715_MIN, I13720_MIN},
    .I_MAX = {I4310_MAX, I6408_MAX, I8112_MAX, I10020_1_MAX, I10020_2_MAX, I13715_MAX, I13720_MAX},
    .KT = {KT4310, KT6408, KT8112, KT10020_1, KT10020_2, KT13715, KT13720}
};

TI5_MOTOR_RANGE ti5_motor_range = {
    .GEAR_RATIO = {
        GEAR_30_40_PRO,GEAR_40_52_PRO, GEAR_50_60_PRO, GEAR_60_70_PRO, GEAR_70_PRO, GEAR_60_PRO_S, GEAR_70_PRO_S
    },
    .V_MAX = {V_30_40_PRO,V_40_52_PRO, V_50_60_PRO, V_60_70_PRO,V_70_PRO,V_60_PRO_S, V_70_PRO_S},
    .I_MAX = {I_30_40_PRO,I_40_52_PRO, I_50_60_PRO, I_60_70_PRO, I_70_PRO, I_60_PRO_S, I_70_PRO_S},
    .T_MAX = {T_30_40_PRO,T_40_52_PRO, T_50_60_PRO, T_60_70_PRO, T_70_PRO, T_60_PRO_S, T_70_PRO_S},
    .TC = {TC_30_40_PRO,TC_40_52_PRO, TC_50_60_PRO, TC_60_70_PRO, TC_70_PRO, TC_60_PRO_S, TC_70_PRO_S},
};
//int Z1_YKS_MOTOR_ID_Type[6] = {A13720, A10020_1, A8112, A13715, A6408, A6408};//这个是Z1的第一版机器人的腿部电机的顺序
//int Z1_YKS_MOTOR_ID_Type[6] = {A13715, A10020_2, A10020_1, A13720, A8112, A8112};//这个是Z1.5机器人的腿部电机的顺序
//这个要根据实际的Z1机器人电机型号来设置，global_id从0开始，对应上层48电机顺序。
int Z1_MOTOR_ID_Type[TOTAL_MOTOR_NUMBER] = {
    A13715, A10020_2, A10020_1, A13720, A8112, A8112, //下肢 左腿
    A13715, A10020_2, A10020_1, A13720, A8112, A8112, //下肢 右腿
    A8112, A6408, A6408, A4310, A4310, A4310, //上肢 左臂
    A8112, A6408, A6408, A4310, A4310, A4310, //上肢 右臂
    A8112, A8112, A8112, A10020_1, A10020_1, A10020_1, //两肩和腰部
    A8112, A8112, A8112, A8112, A8112, A8112, //扩展电机 30~35
    A8112, A8112, A8112, A8112, A8112, A8112, //扩展电机 36~41
    A8112, A8112, A8112, A8112, A8112, A8112 //扩展电机 42~47
};

// 新EtherCAT-CANFD从站映射：
// 从站1使用槽位1~24，对应CAN ID 1~24；从站2使用槽位1~24，对应CAN ID 25~48。
// 每个从站槽位1~8为CANFD1，9~16为CANFD2，17~24为CANFD3。
Motor g_motor_map[TOTAL_MOTOR_NUMBER] = {
    {MOTOR_YKS, 0, 1, 1, 0},
    {MOTOR_YKS, 0, 2, 2, 1},
    {MOTOR_YKS, 0, 3, 3, 2},
    {MOTOR_YKS, 0, 4, 4, 3},
    {MOTOR_YKS, 0, 5, 5, 4},
    {MOTOR_YKS, 0, 6, 6, 5},
    {MOTOR_YKS, 0, 7, 7, 6},
    {MOTOR_YKS, 0, 8, 8, 7},
    {MOTOR_YKS, 0, 9, 9, 8},
    {MOTOR_YKS, 0, 10, 10, 9},
    {MOTOR_YKS, 0, 11, 11, 10},
    {MOTOR_YKS, 0, 12, 12, 11},
    {MOTOR_YKS, 0, 13, 13, 12},
    {MOTOR_YKS, 0, 14, 14, 13},
    {MOTOR_YKS, 0, 15, 15, 14},
    {MOTOR_YKS, 0, 16, 16, 15},
    {MOTOR_YKS, 0, 17, 17, 16},
    {MOTOR_YKS, 0, 18, 18, 17},
    {MOTOR_YKS, 0, 19, 19, 18},
    {MOTOR_YKS, 0, 20, 20, 19},
    {MOTOR_YKS, 0, 21, 21, 20},
    {MOTOR_YKS, 0, 22, 22, 21},
    {MOTOR_YKS, 0, 23, 23, 22},
    {MOTOR_YKS, 0, 24, 24, 23},
    {MOTOR_YKS, 1, 1, 25, 24},
    {MOTOR_YKS, 1, 2, 26, 25},
    {MOTOR_YKS, 1, 3, 27, 26},
    {MOTOR_YKS, 1, 4, 28, 27},
    {MOTOR_YKS, 1, 5, 29, 28},
    {MOTOR_YKS, 1, 6, 30, 29},
    {MOTOR_YKS, 1, 7, 31, 30},
    {MOTOR_YKS, 1, 8, 32, 31},
    {MOTOR_YKS, 1, 9, 33, 32},
    {MOTOR_YKS, 1, 10, 34, 33},
    {MOTOR_YKS, 1, 11, 35, 34},
    {MOTOR_YKS, 1, 12, 36, 35},
    {MOTOR_YKS, 1, 13, 37, 36},
    {MOTOR_YKS, 1, 14, 38, 37},
    {MOTOR_YKS, 1, 15, 39, 38},
    {MOTOR_YKS, 1, 16, 40, 39},
    {MOTOR_YKS, 1, 17, 41, 40},
    {MOTOR_YKS, 1, 18, 42, 41},
    {MOTOR_YKS, 1, 19, 43, 42},
    {MOTOR_YKS, 1, 20, 44, 43},
    {MOTOR_YKS, 1, 21, 45, 44},
    {MOTOR_YKS, 1, 22, 46, 45},
    {MOTOR_YKS, 1, 23, 47, 46},
    {MOTOR_YKS, 1, 24, 48, 47},
};

Slave g_slaves[SLAVE_NUMBER] = {
    {
        .slave_id = 1,
        .motor_count = 24,
        .motors = {
            {MOTOR_YKS, 0, 1, 1, 0},
            {MOTOR_YKS, 0, 2, 2, 1},
            {MOTOR_YKS, 0, 3, 3, 2},
            {MOTOR_YKS, 0, 4, 4, 3},
            {MOTOR_YKS, 0, 5, 5, 4},
            {MOTOR_YKS, 0, 6, 6, 5},
            {MOTOR_YKS, 0, 7, 7, 6},
            {MOTOR_YKS, 0, 8, 8, 7},
            {MOTOR_YKS, 0, 9, 9, 8},
            {MOTOR_YKS, 0, 10, 10, 9},
            {MOTOR_YKS, 0, 11, 11, 10},
            {MOTOR_YKS, 0, 12, 12, 11},
            {MOTOR_YKS, 0, 13, 13, 12},
            {MOTOR_YKS, 0, 14, 14, 13},
            {MOTOR_YKS, 0, 15, 15, 14},
            {MOTOR_YKS, 0, 16, 16, 15},
            {MOTOR_YKS, 0, 17, 17, 16},
            {MOTOR_YKS, 0, 18, 18, 17},
            {MOTOR_YKS, 0, 19, 19, 18},
            {MOTOR_YKS, 0, 20, 20, 19},
            {MOTOR_YKS, 0, 21, 21, 20},
            {MOTOR_YKS, 0, 22, 22, 21},
            {MOTOR_YKS, 0, 23, 23, 22},
            {MOTOR_YKS, 0, 24, 24, 23},
        }
    },
    {
        .slave_id = 2,
        .motor_count = 24,
        .motors = {
            {MOTOR_YKS, 1, 1, 25, 24},
            {MOTOR_YKS, 1, 2, 26, 25},
            {MOTOR_YKS, 1, 3, 27, 26},
            {MOTOR_YKS, 1, 4, 28, 27},
            {MOTOR_YKS, 1, 5, 29, 28},
            {MOTOR_YKS, 1, 6, 30, 29},
            {MOTOR_YKS, 1, 7, 31, 30},
            {MOTOR_YKS, 1, 8, 32, 31},
            {MOTOR_YKS, 1, 9, 33, 32},
            {MOTOR_YKS, 1, 10, 34, 33},
            {MOTOR_YKS, 1, 11, 35, 34},
            {MOTOR_YKS, 1, 12, 36, 35},
            {MOTOR_YKS, 1, 13, 37, 36},
            {MOTOR_YKS, 1, 14, 38, 37},
            {MOTOR_YKS, 1, 15, 39, 38},
            {MOTOR_YKS, 1, 16, 40, 39},
            {MOTOR_YKS, 1, 17, 41, 40},
            {MOTOR_YKS, 1, 18, 42, 41},
            {MOTOR_YKS, 1, 19, 43, 42},
            {MOTOR_YKS, 1, 20, 44, 43},
            {MOTOR_YKS, 1, 21, 45, 44},
            {MOTOR_YKS, 1, 22, 46, 45},
            {MOTOR_YKS, 1, 23, 47, 46},
            {MOTOR_YKS, 1, 24, 48, 47},
        }
    }
};

union RV_TypeConvert {
    float to_float;
    int to_int;
    unsigned int to_uint;
    uint8_t buf[4];
} rv_type_convert;

union RV_TypeConvert2 {
    int16_t to_int16;
    uint16_t to_uint16;
    uint8_t buf[2];
} rv_type_convert2;

MotorCommFbd motor_comm_fbd;
OD_Motor_Msg rv_motor_msg[TOTAL_MOTOR_NUMBER];
IMU_Msg imu_msg;

static int valid_pdo_slot(uint8_t pdo_slot);
static Motor_Msg *pdo_frame(EtherCAT_Msg *msg, uint8_t pdo_slot);
static int find_global_id_by_can_id(const Slave *slave, uint16_t can_id);
static int find_slave_motor_by_slot(const Slave *slave, uint8_t pdo_slot);
static int resolve_global_id(const Slave *slave, uint16_t can_id, uint8_t pdo_slot);
static void set_frame_header(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, uint8_t dlc);

// MOTOR SETTING
/*
cmd:
0x00:NON
0x01:set the communication mode to automatic feedback.
0x02:set the communication mode to response.
0x03:set the current position to zero.
*/
void MotorSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint8_t cmd) {
    // EtherCAT_Msg TxMessage;

    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].dlc = 4;

    if (cmd == 0)
        return;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = cmd;
}

// Reset Motor ID
void MotorIDReset(EtherCAT_Msg *TxMessage) {
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = 0x7F;
    TxMessage->motor[0].data[1] = 0x7F;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x05;
    TxMessage->motor[0].data[4] = 0x7F;
    TxMessage->motor[0].data[5] = 0x7F;
}

// set motor new ID
void MotorIDSetting(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new) {
    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x04;
    TxMessage->motor[0].data[4] = motor_id_new >> 8;
    TxMessage->motor[0].data[5] = motor_id_new & 0xff;
}

// read motor communication mode
void MotorCommModeReading(EtherCAT_Msg *TxMessage, uint16_t motor_id) {
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xff;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x81;
}

// read motor ID
void MotorIDReading(EtherCAT_Msg *TxMessage) {
    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = 0xFF;
    TxMessage->motor[0].data[1] = 0xFF;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x82;
}

// This function use in ask communication mode.
void
send_motor_ctrl_cmd(EtherCAT_Msg *TxMessage, const uint8_t pdo_slot, const uint16_t can_id,
                    const uint16_t global_id, float kp, float kd,
                    float pos,
                    float spd, float cur) {
    if (global_id >= TOTAL_MOTOR_NUMBER || !valid_pdo_slot(pdo_slot))
        return;

    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 8);


    const float KD_MIN = yks_motor_range.KD_MIN[Z1_MOTOR_ID_Type[global_id]];
    const float KD_MAX = yks_motor_range.KD_MAX[Z1_MOTOR_ID_Type[global_id]];
    const float T_MIN = yks_motor_range.T_MIN[Z1_MOTOR_ID_Type[global_id]];
    const float T_MAX = yks_motor_range.T_MAX[Z1_MOTOR_ID_Type[global_id]];

    //电机型号限位
    if (kp > KP_MAX)
        kp = KP_MAX;
    else if (kp < KP_MIN)
        kp = KP_MIN;
    if (kd > KD_MAX)
        kd = KD_MAX;
    else if (kd < KD_MIN)
        kd = KD_MIN;
    if (pos > POS_MAX)
        pos = POS_MAX;
    else if (pos < POS_MIN)
        pos = POS_MIN;
    if (spd > SPD_MAX)
        spd = SPD_MAX;
    else if (spd < SPD_MIN)
        spd = SPD_MIN;
    if (cur > T_MAX)
        cur = T_MAX;
    else if (cur < T_MIN)
        cur = T_MIN;

    //关节限位
    if (pos > Z1_MOTOR_POS_MAX[global_id])
        pos = Z1_MOTOR_POS_MAX[global_id];
    else if (pos < Z1_MOTOR_POS_MIN[global_id])
        pos = Z1_MOTOR_POS_MIN[global_id];
    if (spd > Z1_MOTOR_SPE_MAX[global_id])
        spd = Z1_MOTOR_SPE_MAX[global_id];
    else if (spd < Z1_MOTOR_SPE_MIN[global_id])
        spd = Z1_MOTOR_SPE_MIN[global_id];
    if (cur > Z1_MOTOR_TOR_MAX[global_id])
        cur = Z1_MOTOR_TOR_MAX[global_id];
    else if (cur < Z1_MOTOR_TOR_MIN[global_id])
        cur = Z1_MOTOR_TOR_MIN[global_id];



    const int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    const int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 9);
    const int pos_int = float_to_uint(pos, POS_MIN, POS_MAX, 16);
    const int spd_int = float_to_uint(spd, SPD_MIN, SPD_MAX, 12);
    const int tor_int = float_to_uint(cur, T_MIN, T_MAX, 12);

    frame->data[0] = 0x00 | (kp_int >> 7); // kp5
    frame->data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8); // kp7+kd1
    frame->data[2] = kd_int & 0xFF;
    frame->data[3] = pos_int >> 8;
    frame->data[4] = pos_int & 0xFF;
    frame->data[5] = spd_int >> 4;
    frame->data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    frame->data[7] = tor_int & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
pos:float
spd:0~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_position(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, float pos, uint16_t spd,
                        uint16_t cur, uint8_t ack_status) {
    if (!valid_pdo_slot(pdo_slot))
        return;

    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 8);

    if (ack_status > 3)
        return;

    rv_type_convert.to_float = pos;
    frame->data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    frame->data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    frame->data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    frame->data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    frame->data[4] = (rv_type_convert.buf[0] << 5) | (spd >> 10);
    frame->data[5] = (spd & 0x3FC) >> 2;
    frame->data[6] = (spd & 0x03) << 6 | (cur >> 6);
    frame->data[7] = (cur & 0x3F) << 2 | ack_status;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
spd:-18000~18000
cur:0~3000
ack_status:0~3
*/
void set_motor_speed(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, float spd, uint16_t cur,
                     uint8_t ack_status) {
    if (!valid_pdo_slot(pdo_slot))
        return;

    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 7);

    rv_type_convert.to_float = spd;
    frame->data[0] = 0x40 | ack_status;
    frame->data[1] = rv_type_convert.buf[3];
    frame->data[2] = rv_type_convert.buf[2];
    frame->data[3] = rv_type_convert.buf[1];
    frame->data[4] = rv_type_convert.buf[0];
    frame->data[5] = cur >> 8;
    frame->data[6] = cur & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
cur:-3000~3000
ctrl_status:
    0:current control
    1:torque control
    2:variable damping brake control(also call full brake)
    3:dynamic brake control
    4:regenerative brake control
    5:NON
    6:NON
    7:NON
ack_status:0~3
*/
void set_motor_cur_tor(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, int16_t cur_tor,
                       uint8_t ctrl_status, uint8_t ack_status) {
    if (!valid_pdo_slot(pdo_slot))
        return;
    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 3);

    if (ack_status > 3)
        return;
    if (ctrl_status > 7)
        return;
    if (ctrl_status) // enter torque control mode or brake mode
    {
        if (cur_tor > 3000)
            cur_tor = 3000;
        else if (cur_tor < -3000)
            cur_tor = -3000;
    } else {
        if (cur_tor > 2000)
            cur_tor = 2000;
        else if (cur_tor < -2000)
            cur_tor = -2000;
    }

    frame->data[0] = 0x60 | ctrl_status << 2 | ack_status;
    frame->data[1] = cur_tor >> 8;
    frame->data[2] = cur_tor & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
acc:0~2000
ack_status:0~3
*/
void set_motor_acceleration(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, uint16_t acc,
                            uint8_t ack_status) {
    if (!valid_pdo_slot(pdo_slot))
        return;
    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 4);

    if (ack_status > 2)
        return;
    if (acc > 2000)
        acc = 2000;

    frame->data[0] = 0xC0 | ack_status;
    frame->data[1] = 0x01;
    frame->data[2] = acc >> 8;
    frame->data[3] = acc & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
linkage:0~10000
speedKI:0~10000
ack_status:0/1
*/
void set_motor_linkage_speedKI(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, uint16_t linkage,
                               uint16_t speedKI, uint8_t ack_status) {
    if (!valid_pdo_slot(pdo_slot))
        return;
    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 6);

    if (ack_status > 2)
        return;
    if (linkage > 10000)
        linkage = 10000;
    if (speedKI > 10000)
        speedKI = 10000;

    frame->data[0] = 0xC0 | ack_status;
    frame->data[1] = 0x02;
    frame->data[2] = linkage >> 8;
    frame->data[3] = linkage & 0xff;
    frame->data[4] = speedKI >> 8;
    frame->data[5] = speedKI & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
fdbKP:0~10000
fbdKD:0~10000
ack_status:0/1
*/
void set_motor_feedbackKP_KD(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, uint16_t fdbKP,
                             uint16_t fdbKD, uint8_t ack_status) {
    if (!valid_pdo_slot(pdo_slot))
        return;
    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 6);

    if (ack_status > 2)
        return;
    if (fdbKP > 10000)
        fdbKP = 10000;
    if (fdbKD > 10000)
        fdbKD = 10000;

    frame->data[0] = 0xC0 | ack_status;
    frame->data[1] = 0x03;
    frame->data[2] = fdbKP >> 8;
    frame->data[3] = fdbKP & 0xff;
    frame->data[4] = fdbKD >> 8;
    frame->data[5] = fdbKD & 0xff;
}

// This function use in ask communication mode.
/*
motor_id:1~0x7FE
param_cmd:1~9
*/
void get_motor_parameter(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, uint8_t param_cmd) {
    if (!valid_pdo_slot(pdo_slot))
        return;
    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 2);

    frame->data[0] = 0xE0;
    frame->data[1] = param_cmd;
}

void Rv_Message_Print(uint8_t ack_status) {
    if (ack_status >= 100) {
        printf("motor%d message:\n", ack_status - 100);
        if (motor_comm_fbd.motor_fbd == 0x01) {
            printf("自动模式.\n");
        } else if (motor_comm_fbd.motor_fbd == 0x02) {
            printf("问答模式.\n");
        } else if (motor_comm_fbd.motor_fbd == 0x03) {
            printf("零点设置成功.\n");
        } else if (motor_comm_fbd.motor_fbd == 0x04) {
            printf("新设置的Id为: %d\n", motor_comm_fbd.motor_id);
        } else if (motor_comm_fbd.motor_fbd == 0x05) {
            printf("重置Id成功.\n");
        } else if (motor_comm_fbd.motor_fbd == 0x06) {
            printf("当前电机Id: %d\n", motor_comm_fbd.motor_id);
        } else if (motor_comm_fbd.motor_fbd == 0x80) {
            printf("查询失败.\n");
        }
    } else {
        for (int i = 0; i < TOTAL_MOTOR_NUMBER; ++i) {
            if (rv_motor_msg[i].motor_id == 0) {
                continue;
            }
            switch (ack_status) {
                case 1:
                    printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                    printf("angle_actual_rad: %f\n", rv_motor_msg[i].angle_actual_rad);
                    printf("speed_actual_rad: %f\n", rv_motor_msg[i].speed_actual_rad);
                    printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                    printf("temperature: %d\n", rv_motor_msg[i].temperature);
                    break;
                case 2:
                    printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                    printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                    printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                    printf("temperature: %d\n", rv_motor_msg[i].temperature);
                    printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                    break;
                case 3:
                    printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                    printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                    printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                    printf("temperature: %d\n", rv_motor_msg[i].temperature);
                    printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                    break;
                case 4:
                    if (motor_comm_fbd.motor_fbd == 0) {
                        printf("配置成功.\n");
                    } else {
                        printf("配置失败.\n");
                    }
                    break;
                case 5:
                    printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                    if (motor_comm_fbd.INS_code == 1) {
                        printf("angle_actual_float: %f\n", rv_motor_msg[i].angle_actual_float);
                    } else if (motor_comm_fbd.INS_code == 2) {
                        printf("speed_actual_float: %f\n", rv_motor_msg[i].speed_actual_float);
                    } else if (motor_comm_fbd.INS_code == 3) {
                        printf("current_actual_float: %f\n", rv_motor_msg[i].current_actual_float);
                    } else if (motor_comm_fbd.INS_code == 4) {
                        printf("power: %f\n", rv_motor_msg[i].power);
                    } else if (motor_comm_fbd.INS_code == 5) {
                        printf("acceleration: %d\n", rv_motor_msg[i].acceleration);
                    } else if (motor_comm_fbd.INS_code == 6) {
                        printf("linkage_KP: %d\n", rv_motor_msg[i].linkage_KP);
                    } else if (motor_comm_fbd.INS_code == 7) {
                        printf("speed_KI: %d\n", rv_motor_msg[i].speed_KI);
                    } else if (motor_comm_fbd.INS_code == 8) {
                        printf("feedback_KP: %d\n", rv_motor_msg[i].feedback_KP);
                    } else if (motor_comm_fbd.INS_code == 9) {
                        printf("feedback_KD: %d\n", rv_motor_msg[i].feedback_KD);
                    }
                    break;
                case 6:
                    printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                    printf("angle_actual_int: %d\n", rv_motor_msg[i].angle_actual_int);
                    printf("speed_actual_int: %d\n", rv_motor_msg[i].speed_actual_int);
                    printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                    printf("temperature: %d\n", rv_motor_msg[i].temperature);
                    break;
                default:
                    break;
            }
        }
    }
}

uint16_t motor_id_check = 0;

static int valid_pdo_slot(uint8_t pdo_slot) {
    return pdo_slot >= 1 && pdo_slot <= ACTIVE_MOTOR_NUMBER;
}

static Motor_Msg *pdo_frame(EtherCAT_Msg *msg, uint8_t pdo_slot) {
    if (!valid_pdo_slot(pdo_slot)) {
        return NULL;
    }
    return &msg->motor[pdo_slot - 1];
}

static int find_global_id_by_can_id(const Slave *slave, uint16_t can_id) {
    for (int i = 0; i < slave->motor_count; ++i) {
        if (slave->motors[i].can_id == can_id) {
            return slave->motors[i].global_id;
        }
    }
    return -1;
}

static int find_slave_motor_by_slot(const Slave *slave, uint8_t pdo_slot) {
    for (int i = 0; i < slave->motor_count; ++i) {
        if (slave->motors[i].pdo_slot == pdo_slot) {
            return i;
        }
    }
    return -1;
}

static int resolve_global_id(const Slave *slave, uint16_t can_id, uint8_t pdo_slot) {
    const int by_can_id = find_global_id_by_can_id(slave, can_id);
    if (by_can_id >= 0) {
        return by_can_id;
    }
    const int by_slot = find_slave_motor_by_slot(slave, pdo_slot);
    if (by_slot >= 0) {
        return slave->motors[by_slot].global_id;
    }
    return -1;
}

static void set_frame_header(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint16_t can_id, uint8_t dlc) {
    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    if (frame == NULL) {
        return;
    }
    TxMessage->can_ide = 0;
    frame->id = can_id;
    frame->rtr = 0;
    frame->dlc = dlc;
}

void RV_can_data_repack(const EtherCAT_Msg *RxMessage, const uint8_t comm_mode, const Slave *slave,
                        uint8_t *motor_ack_status) {
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;
    int error_int = 0;
    int mos_temperature_int = 0;

    for (int i = 0; i < slave->motor_count; ++i) {
        const Motor *motor = &slave->motors[i];
        const uint8_t pdo_index = motor->pdo_slot - 1;
        if (motor->pdo_slot < 1 || motor->pdo_slot > ACTIVE_MOTOR_NUMBER) {
            continue;
        }
        const Motor_Msg *frame = &RxMessage->motor[pdo_index];
        int global_id = motor->global_id;
        if (frame->id != 0x7FF) {
            const int resolved = resolve_global_id(slave, frame->id, motor->pdo_slot);
            if (resolved >= 0) {
                global_id = resolved;
            }
        }
        if (global_id < 0 || global_id >= TOTAL_MOTOR_NUMBER) {
            continue;
        }
        if (motor->type == MOTOR_YKS) {
            const float I_MIN = yks_motor_range.I_MIN[Z1_MOTOR_ID_Type[global_id]];
            const float I_MAX = yks_motor_range.I_MAX[Z1_MOTOR_ID_Type[global_id]];
            const float KT = yks_motor_range.KT[Z1_MOTOR_ID_Type[global_id]];
            if (frame->dlc == 0)
                continue;
            // printf("motor%d:\n", i + 1);
            if (frame->id == 0x7FF) {
                if (frame->data[2] != 0x01) // determine whether it is a motor feedback instruction
                {
                    motor_ack_status[global_id] = 255;
                    // printf("motor%d: not a motor feedback instruction\n", i + 1);
                    continue;
                    // return 255; // it is not a motor feedback instruction
                }

                if ((frame->data[0] == 0xff) && (frame->data[1] == 0xFF)) {
                    motor_comm_fbd.motor_id = frame->data[3] << 8 | frame->data[4];
                    motor_comm_fbd.motor_fbd = 0x06;
                } else if ((frame->data[0] == 0x80) && (frame->data[1] == 0x80))
                // inquire failed
                {
                    motor_comm_fbd.motor_id = 0;
                    motor_comm_fbd.motor_fbd = 0x80;
                } else if ((frame->data[0] == 0x7F) &&
                           (frame->data[1] == 0x7F)) // reset ID succeed
                {
                    motor_comm_fbd.motor_id = 1;
                    motor_comm_fbd.motor_fbd = 0x05;
                } else {
                    motor_comm_fbd.motor_id = frame->data[0] << 8 | frame->data[1];
                    motor_comm_fbd.motor_fbd = frame->data[3];
                }
                motor_ack_status[global_id] = 100 + pdo_index;
                // return 100 + i;
            } else if (comm_mode == 0x00 && frame->dlc != 0) // Response mode
            {
                // printf("id = %d\n"i);
                ack_status = frame->data[0] >> 5;
                motor_id_check = frame->id;
                rv_motor_msg[global_id].motor_id = motor_id_check;
                rv_motor_msg[global_id].error = frame->data[0] & 0x1F;
                if (ack_status == 1) // response frame 1
                {
                    pos_int = frame->data[1] << 8 | frame->data[2];
                    spd_int = frame->data[3] << 4 | (frame->data[4] & 0xF0) >> 4;
                    cur_int = (frame->data[4] & 0x0F) << 8 | frame->data[5];
                    error_int = frame->data[0] & 0x1F;
                    mos_temperature_int = frame->data[7];

                    rv_motor_msg[global_id].angle_actual_rad = uint_to_float(pos_int, POS_MIN, POS_MAX, 16);
                    rv_motor_msg[global_id].speed_actual_rad = uint_to_float(spd_int, SPD_MIN, SPD_MAX, 12);
                    rv_motor_msg[global_id].current_actual_float = KT * uint_to_float(cur_int, I_MIN, I_MAX, 12);
                    //这里我将电流乘以KT转变为力矩，所以实际上这里是力矩
                    rv_motor_msg[global_id].temperature = (frame->data[6] - 50) / 2;
                    rv_motor_msg[global_id].error = error_int;
                    rv_motor_msg[global_id].mos_temperature = (mos_temperature_int - 50) / 2;
                } else if (ack_status == 2) // response frame 2
                {
                    rv_type_convert.buf[0] = frame->data[4];
                    rv_type_convert.buf[1] = frame->data[3];
                    rv_type_convert.buf[2] = frame->data[2];
                    rv_type_convert.buf[3] = frame->data[1];
                    rv_motor_msg[global_id].angle_actual_float = rv_type_convert.to_float;
                    rv_motor_msg[global_id].current_actual_int =
                            frame->data[5] << 8 | frame->data[6];
                    rv_motor_msg[global_id].temperature = (frame->data[7] - 50) / 2;
                    rv_motor_msg[global_id].current_actual_float = rv_motor_msg[global_id].current_actual_int /
                                                                    100.0f;
                } else if (ack_status == 3) // response frame 3
                {
                    rv_type_convert.buf[0] = frame->data[4];
                    rv_type_convert.buf[1] = frame->data[3];
                    rv_type_convert.buf[2] = frame->data[2];
                    rv_type_convert.buf[3] = frame->data[1];
                    rv_motor_msg[global_id].speed_actual_float = rv_type_convert.to_float;
                    rv_motor_msg[global_id].current_actual_int =
                            frame->data[5] << 8 | frame->data[6];
                    rv_motor_msg[global_id].temperature = (frame->data[7] - 50) / 2;


                    rv_motor_msg[global_id].current_actual_float = rv_motor_msg[global_id].current_actual_int /
                                                                    100.0f;
                } else if (ack_status == 4) // response frame 4
                {
                    if (frame->dlc != 3) {
                        motor_ack_status[global_id] = 255;
                        continue;
                        // return 255;
                    }
                    motor_comm_fbd.INS_code = frame->data[1];
                    motor_comm_fbd.motor_fbd = frame->data[2];
                } else if (ack_status == 5) // response frame 5
                {
                    motor_comm_fbd.INS_code = frame->data[1];
                    if (motor_comm_fbd.INS_code == 1 && frame->dlc == 6) // get position
                    {
                        rv_type_convert.buf[0] = frame->data[5];
                        rv_type_convert.buf[1] = frame->data[4];
                        rv_type_convert.buf[2] = frame->data[3];
                        rv_type_convert.buf[3] = frame->data[2];
                        rv_motor_msg[global_id].angle_actual_float = rv_type_convert.to_float;
                    } else if (motor_comm_fbd.INS_code == 2 && frame->dlc == 6) // get speed
                    {
                        rv_type_convert.buf[0] = frame->data[5];
                        rv_type_convert.buf[1] = frame->data[4];
                        rv_type_convert.buf[2] = frame->data[3];
                        rv_type_convert.buf[3] = frame->data[2];
                        rv_motor_msg[global_id].speed_actual_float = rv_type_convert.to_float;
                    } else if (motor_comm_fbd.INS_code == 3 && frame->dlc == 6) // get current
                    {
                        rv_type_convert.buf[0] = frame->data[5];
                        rv_type_convert.buf[1] = frame->data[4];
                        rv_type_convert.buf[2] = frame->data[3];
                        rv_type_convert.buf[3] = frame->data[2];
                        rv_motor_msg[global_id].current_actual_float = rv_type_convert.to_float;
                    } else if (motor_comm_fbd.INS_code == 4 && frame->dlc == 6) // get power
                    {
                        rv_type_convert.buf[0] = frame->data[5];
                        rv_type_convert.buf[1] = frame->data[4];
                        rv_type_convert.buf[2] = frame->data[3];
                        rv_type_convert.buf[3] = frame->data[2];
                        rv_motor_msg[global_id].power = rv_type_convert.to_float;
                    } else if (motor_comm_fbd.INS_code == 5 && frame->dlc == 4) // get acceleration
                    {
                        rv_motor_msg[global_id].acceleration =
                                frame->data[2] << 8 | frame->data[3];
                    } else if (motor_comm_fbd.INS_code == 6 && frame->dlc == 4) // get linkage_KP
                    {
                        rv_motor_msg[global_id].linkage_KP =
                                frame->data[2] << 8 | frame->data[3];
                    } else if (motor_comm_fbd.INS_code == 7 && frame->dlc == 4) // get speed_KI
                    {
                        rv_motor_msg[global_id].speed_KI = frame->data[2] << 8 | frame->data[
                                                                 3];
                    } else if (motor_comm_fbd.INS_code == 8 && frame->dlc == 4) // get feedback_KP
                    {
                        rv_motor_msg[global_id].feedback_KP =
                                frame->data[2] << 8 | frame->data[3];
                    } else if (motor_comm_fbd.INS_code == 9 && frame->dlc == 4) // get feedback_KD
                    {
                        rv_motor_msg[global_id].feedback_KD =
                                frame->data[2] << 8 | frame->data[3];
                    }
                }
                motor_ack_status[global_id] = ack_status;

                // return ack_status;
            } else if (comm_mode == 0x01 && frame->dlc != 0) // automatic feedback mode
            {
                rv_motor_msg[global_id].motor_id = frame->id;
                rv_motor_msg[global_id].angle_actual_int = (uint16_t) (frame->data[0] << 8 |
                                                                        frame->data[1]);
                rv_motor_msg[global_id].speed_actual_int = (int16_t) (frame->data[2] << 8 |
                                                                       frame->data[3]);
                rv_motor_msg[global_id].current_actual_int = (frame->data[4] << 8 |
                                                               frame->data[5]);
                rv_motor_msg[global_id].temperature = frame->data[6];
                rv_motor_msg[global_id].error = frame->data[7];
                motor_ack_status[global_id] = 6;
                // return 6;
            }
        } else if (motor->type == MOTOR_TI5) {
            if (frame->dlc == 0) {
                continue;
            }
            const float GEAR_RATIO = ti5_motor_range.GEAR_RATIO[Z1_MOTOR_ID_Type[global_id]];
            // printf("motor%d:\n", i + 1);
            if (frame->dlc != 0) // Response mode
            {
                motor_id_check = frame->id;

                rv_motor_msg[global_id].motor_id = motor_id_check;

                uint16_t ti5_cur_uint = frame->data[0] + (frame->data[1] << 8);
                uint16_t ti5_spd_uint = frame->data[2] + (frame->data[3] << 8);
                uint32_t ti5_pos_uint = frame->data[4] + (frame->data[5] << 8) + (
                                            frame->data[6] << 16) + (
                                            frame->data[7] << 24);

                int16_t ti5_cur_int;
                int16_t ti5_spd_int;
                int32_t ti5_pos_int;

                memcpy(&ti5_cur_int, &ti5_cur_uint, sizeof(uint16_t));
                memcpy(&ti5_spd_int, &ti5_spd_uint, sizeof(uint16_t));
                memcpy(&ti5_pos_int, &ti5_pos_uint, sizeof(uint32_t));

                float cur_float = ti5_cur_int;
                float spd_float = ti5_spd_int;
                double pos_double = ti5_pos_int;

                rv_motor_msg[global_id].current_actual_float =
                        cur_float * ti5_motor_range.TC[Z1_MOTOR_ID_Type[global_id]];
                rv_motor_msg[global_id].speed_actual_rad = (spd_float * 360.0f) / (GEAR_RATIO * 100 * 57.2958f);
                rv_motor_msg[global_id].angle_actual_rad = (pos_double * 360.0f) / (65536 * GEAR_RATIO * 57.2958f);

                motor_ack_status[global_id] = 1;
                // printf("CUR = %f ", rv_motor_msg[motor_id_t].current_actual_float);
                // printf("SPD = %f ", rv_motor_msg[motor_id_t].speed_actual_float);
                // printf("POS = %lf \n", rv_motor_msg[motor_id_t].angle_actual_float);
            }
        }

        // printf("i = %d\n", i);
        // printf("id: %d\n", RxMessage->motor[i].id);
        // printf("motor dlc%d:\n", RxMessage->motor[i].dlc);

        // printf("Motor %d: %d\n", i, motor_ack_status[i]);
    }
}

void RV_can_imu_data_repack(EtherCAT_Msg *RxMessage) {
    uint16_t angle_uint[3], gyro_uint[3], accel_uint[3], mag_uint[3];

    rv_type_convert2.buf[0] = RxMessage->motor[0].data[1];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[0];
    angle_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[0].data[3];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[2];
    angle_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[0].data[5];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[4];
    angle_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert2.buf[0] = RxMessage->motor[0].data[7];
    rv_type_convert2.buf[1] = RxMessage->motor[0].data[6];
    gyro_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[1].data[1];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[0];
    gyro_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[1].data[3];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[2];
    gyro_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert2.buf[0] = RxMessage->motor[1].data[5];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[4];
    accel_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[1].data[7];
    rv_type_convert2.buf[1] = RxMessage->motor[1].data[6];
    accel_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[2].data[1];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[0];
    accel_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert2.buf[0] = RxMessage->motor[2].data[3];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[2];
    mag_uint[0] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[2].data[5];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[4];
    mag_uint[1] = rv_type_convert2.to_uint16;
    rv_type_convert2.buf[0] = RxMessage->motor[2].data[7];
    rv_type_convert2.buf[1] = RxMessage->motor[2].data[6];
    mag_uint[2] = rv_type_convert2.to_uint16;

    rv_type_convert.buf[0] = RxMessage->motor[3].data[3];
    rv_type_convert.buf[1] = RxMessage->motor[3].data[2];
    rv_type_convert.buf[2] = RxMessage->motor[3].data[1];
    rv_type_convert.buf[3] = RxMessage->motor[3].data[0];

    imu_msg.quat_float[0] = rv_type_convert.to_float;
    rv_type_convert.buf[0] = RxMessage->motor[3].data[7];
    rv_type_convert.buf[1] = RxMessage->motor[3].data[6];
    rv_type_convert.buf[2] = RxMessage->motor[3].data[5];
    rv_type_convert.buf[3] = RxMessage->motor[3].data[4];

    imu_msg.quat_float[1] = rv_type_convert.to_float;
    rv_type_convert.buf[0] = RxMessage->motor[4].data[3];
    rv_type_convert.buf[1] = RxMessage->motor[4].data[2];
    rv_type_convert.buf[2] = RxMessage->motor[4].data[1];
    rv_type_convert.buf[3] = RxMessage->motor[4].data[0];

    imu_msg.quat_float[2] = rv_type_convert.to_float;
    rv_type_convert.buf[0] = RxMessage->motor[4].data[7];
    rv_type_convert.buf[1] = RxMessage->motor[4].data[6];
    rv_type_convert.buf[2] = RxMessage->motor[4].data[5];
    rv_type_convert.buf[3] = RxMessage->motor[4].data[4];

    imu_msg.quat_float[3] = rv_type_convert.to_float;

    imu_msg.angle_float[0] = uint_to_float(angle_uint[0], -3.5, 3.5, 16);
    imu_msg.angle_float[1] = uint_to_float(angle_uint[1], -3.5, 3.5, 16);
    imu_msg.angle_float[2] = uint_to_float(angle_uint[2], -3.5, 3.5, 16);

    imu_msg.gyro_float[0] = uint_to_float(gyro_uint[0], -35.0, 35.0, 16);
    imu_msg.gyro_float[1] = uint_to_float(gyro_uint[1], -35.0, 35.0, 16);
    imu_msg.gyro_float[2] = uint_to_float(gyro_uint[2], -35.0, 35.0, 16);

    imu_msg.accel_float[0] = uint_to_float(accel_uint[0], -240.0, 240.0, 16);
    imu_msg.accel_float[1] = uint_to_float(accel_uint[1], -240.0, 240.0, 16);
    imu_msg.accel_float[2] = uint_to_float(accel_uint[2], -240.0, 240.0, 16);

    imu_msg.mag_float[0] = uint_to_float(mag_uint[0], -250.0, 250.0, 16);
    imu_msg.mag_float[1] = uint_to_float(mag_uint[1], -250.0, 250.0, 16);
    imu_msg.mag_float[2] = uint_to_float(mag_uint[2], -250.0, 250.0, 16);
}

// 这个函数用于直接设置TI5电机电流数据封装发送，单位：毫安
/*
motor_id:1~0x7FE
pdo_slot:1~24
spd:-18000~18000
ack_status:0~3
*/
void set_ti5_current(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint32_t can_id, uint16_t global_id, float cur) {
    if (!valid_pdo_slot(pdo_slot) || global_id >= TOTAL_MOTOR_NUMBER)
        return;
    const float I_MAX = ti5_motor_range.I_MAX[Z1_MOTOR_ID_Type[global_id]];

    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 5);

    // 设置指令代码为 0x42
    frame->data[0] = 0x42;

    uint32_t torque_uint = 0;
    double cur_double = clamping(cur, -I_MAX, I_MAX);
    torque_uint = (unsigned int) (cur_double / ti5_motor_range.TC[Z1_MOTOR_ID_Type[global_id]]);
    // printf("torque_uint = %d\n", torque_uint);


    frame->data[1] = (uint8_t) (torque_uint & 0xFF);
    frame->data[2] = (uint8_t) ((torque_uint >> 8) & 0xFF);

    frame->data[3] = (uint8_t) ((torque_uint >> 16) & 0xFF);
    frame->data[4] = (uint8_t) ((torque_uint >> 24) & 0xFF);
}

// 这个函数用于直接设置TI5电机速度数据封装发送，单位：弧度每秒
/*
motor_id:1~0x7FE
pdo_slot:1~24
spd:-18000~18000
ack_status:0~3
*/
void set_ti5_speed(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint32_t can_id, uint16_t global_id, float spd) {
    if (!valid_pdo_slot(pdo_slot) || global_id >= TOTAL_MOTOR_NUMBER)
        return;
    const float V_MAX = ti5_motor_range.V_MAX[Z1_MOTOR_ID_Type[global_id]];
    const int GEAR_RATIO = ti5_motor_range.GEAR_RATIO[Z1_MOTOR_ID_Type[global_id]];

    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 5);

    // 设置指令代码为 0x43
    frame->data[0] = 0x43;

    // 计算公式
    double speed_rps = clamping(spd, -V_MAX, V_MAX);
    // printf("速度 %f\n", speed_rps);
    speed_rps = (speed_rps * GEAR_RATIO * 100 * 57.2958f) / (360.0f);

    uint32_t speed_uint = 0;
    speed_uint = (unsigned int) (speed_rps);

    frame->data[1] = (uint8_t) (speed_uint & 0xFF);
    frame->data[2] = (uint8_t) ((speed_uint >> 8) & 0xFF);

    frame->data[3] = (uint8_t) ((speed_uint >> 16) & 0xFF);
    frame->data[4] = (uint8_t) ((speed_uint >> 24) & 0xFF);
}

// 这个函数用于直接设置TI5电机位置数据封装发送，单位：弧度
/*
motor_id:1~0x7FE
pdo_slot:1~24
pos:-18000~18000
ack_status:0~3
*/
void set_ti5_position(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint32_t can_id, uint16_t global_id, float pos) {
    if (!valid_pdo_slot(pdo_slot) || global_id >= TOTAL_MOTOR_NUMBER)
        return;

    const int GEAR_RATIO = ti5_motor_range.GEAR_RATIO[Z1_MOTOR_ID_Type[global_id]];

    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 5);

    // 设置指令代码为 0x44
    frame->data[0] = 0x44;

    // 计算公式
    double position_rps = (pos / 360.0f) * (GEAR_RATIO * 65536 * 57.2958f);
    // printf("位置 %f\n", position_rps);

    uint32_t position_uint = 0;
    position_uint = (unsigned int) (clamping(position_rps, -174626721.0, 174626721.0));

    frame->data[1] = (uint8_t) (position_uint & 0xFF);
    frame->data[2] = (uint8_t) ((position_uint >> 8) & 0xFF);

    frame->data[3] = (uint8_t) ((position_uint >> 16) & 0xFF);
    frame->data[4] = (uint8_t) ((position_uint >> 24) & 0xFF);
}


// 这个函数用于设置TI5电机刹车
/*
motor_id:1~0x7FE
pdo_slot:1~24
spd:-18000~18000
ack_status:0~3
*/
void set_ti5_stop(EtherCAT_Msg *TxMessage, uint8_t pdo_slot, uint32_t can_id, int times) {
    if (!valid_pdo_slot(pdo_slot))
        return;

    Motor_Msg *frame = pdo_frame(TxMessage, pdo_slot);
    set_frame_header(TxMessage, pdo_slot, can_id, 5);

    // 设置指令代码为 0x44
    frame->data[0] = 0x02;

    uint32_t delay_time = 0;
    delay_time = (unsigned int) (times);


    frame->data[1] = (uint8_t) (delay_time & 0xFF);
    frame->data[2] = (uint8_t) ((delay_time >> 8) & 0xFF);

    frame->data[3] = (uint8_t) ((delay_time >> 16) & 0xFF);
    frame->data[4] = (uint8_t) ((delay_time >> 24) & 0xFF);
}
