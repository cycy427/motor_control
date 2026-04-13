#include "PhyArc.h"

#include <cstdint>
#include <cstring>

namespace {

constexpr uint8_t kPhyArcMinChannel = 1;
constexpr uint8_t kPhyArcMaxChannel = 6;
constexpr uint8_t kPhyArcPayloadBytes = 8;

bool isValidPhyArcChannel(uint8_t data_channel) {
    return data_channel >= kPhyArcMinChannel && data_channel <= kPhyArcMaxChannel;
}

bool preparePhyArcSlot(EtherCAT_Msg *tx_message, uint8_t data_channel, uint32_t motor_id, uint8_t dlc) {
    if (tx_message == nullptr || !isValidPhyArcChannel(data_channel) || dlc > kPhyArcPayloadBytes) {
        return false;
    }

    Motor_Msg &motor_slot = tx_message->motor[data_channel - 1];
    tx_message->can_ide = 0;
    motor_slot.rtr = 0;
    motor_slot.id = motor_id;
    motor_slot.dlc = dlc;
    std::memset(motor_slot.data, 0, sizeof(motor_slot.data));
    return true;
}

void clearPhyArcSlot(EtherCAT_Msg *tx_message, uint8_t data_channel) {
    if (tx_message == nullptr || !isValidPhyArcChannel(data_channel)) {
        return;
    }

    Motor_Msg &motor_slot = tx_message->motor[data_channel - 1];
    motor_slot.id = 0;
    motor_slot.rtr = 0;
    motor_slot.dlc = 0;
    std::memset(motor_slot.data, 0, sizeof(motor_slot.data));
}

uint8_t mapPhyArcMode(int requested_mode) {
    if (requested_mode == 1) {      // 位置模式
        return 0x03;
    }
    else if (requested_mode == 2) {      // 电流模式
        return 0x01;
    }
    else if (requested_mode == 3) {      // 速度模式
        return 0x02;
    }

    return 0x00; 
}

uint32_t floatToBigEndianHex(float value) {
    uint32_t result = 0;

    std::memcpy(&result, &value, sizeof(float));
    
    uint16_t checkEndian = 0x0001;
    bool isLittleEndian = (*reinterpret_cast<uint8_t*>(&checkEndian) == 0x01);
    
    if (isLittleEndian) {
        result = ((result & 0xFF000000) >> 24) |
                 ((result & 0x00FF0000) >> 8)  |
                 ((result & 0x0000FF00)) << 8  |
                 ((result & 0x000000FF) << 24);
    }
    // 如果是大端机器，则不需要做任何操作
    
    return result;
}


} // namespace

extern "C" void set_phyarc_enable(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, bool on) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 8)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x03;
    motor_slot.data[3] = 0x06;
    motor_slot.data[4] = 0x00;
    motor_slot.data[5] = 0x00;
    motor_slot.data[6] = 0xF0;
    motor_slot.data[7] = on ? 0x02 : 0x01;
}

extern "C" void set_phyarc_mode(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, uint8_t mode) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 8)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x02;
    motor_slot.data[3] = 0x06;
    motor_slot.data[4] = 0x00;
    motor_slot.data[5] = 0x00;
    motor_slot.data[6] = 0x00;
    motor_slot.data[7] = mode;
}

extern "C" void set_phyarc_id(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, uint8_t new_id, bool broadcast) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 8)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = broadcast ? 0xFE : static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x01;
    motor_slot.data[3] = 0x06;
    motor_slot.data[4] = 0x00;
    motor_slot.data[5] = 0x00;
    motor_slot.data[6] = 0x00;
    motor_slot.data[7] = new_id;
}

// 这个设置零点的配置需要在配置后等待500ms以上才能安全生效
extern "C" void set_phyarc_zeropoint(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id ) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 8)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x03;
    motor_slot.data[3] = 0x06;
    motor_slot.data[4] = 0x00;
    motor_slot.data[5] = 0x00;
    motor_slot.data[6] = 0x10;
    motor_slot.data[7] = 0x4F;
}

extern "C" void set_phyarc_posP_spdPI(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float posKP, float spdKP, float spdKI) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 16)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x0B;
    motor_slot.data[3] = 0x06;

    uint32_t posKP_hex = floatToBigEndianHex(posKP);
    uint32_t spdKP_hex = floatToBigEndianHex(spdKP);
    uint32_t spdKI_hex = floatToBigEndianHex(spdKI);

    motor_slot.data[4] = (posKP_hex >> 24) & 0xFF;
    motor_slot.data[5] = (posKP_hex >> 16) & 0xFF;
    motor_slot.data[6] = (posKP_hex >> 8) & 0xFF;
    motor_slot.data[7] = posKP_hex & 0xFF;

    motor_slot.data[8] = (spdKP_hex >> 24) & 0xFF;
    motor_slot.data[9] = (spdKP_hex >> 16) & 0xFF;
    motor_slot.data[10] = (spdKP_hex >> 8) & 0xFF;
    motor_slot.data[11] = spdKP_hex & 0xFF;

    motor_slot.data[12] = (spdKI_hex >> 24) & 0xFF;
    motor_slot.data[13] = (spdKI_hex >> 16) & 0xFF;
    motor_slot.data[14] = (spdKI_hex >> 8) & 0xFF;
    motor_slot.data[15] = spdKI_hex & 0xFF;
}


extern "C" void set_phyarc_TorAPosContorl(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float pos, float spd, float tor) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 16)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x07;
    motor_slot.data[3] = 0x06;

    uint32_t pos_hex = floatToBigEndianHex(pos);
    uint32_t spd_hex = floatToBigEndianHex(spd);
    uint32_t tor_hex = floatToBigEndianHex(tor);

    motor_slot.data[4] = (pos_hex >> 24) & 0xFF;
    motor_slot.data[5] = (pos_hex >> 16) & 0xFF;
    motor_slot.data[6] = (pos_hex >> 8) & 0xFF;
    motor_slot.data[7] = pos_hex & 0xFF;

    motor_slot.data[8] = (spd_hex >> 24) & 0xFF;
    motor_slot.data[9] = (spd_hex >> 16) & 0xFF;
    motor_slot.data[10] = (spd_hex >> 8) & 0xFF;
    motor_slot.data[11] = spd_hex & 0xFF;

    motor_slot.data[12] = (tor_hex >> 24) & 0xFF;
    motor_slot.data[13] = (tor_hex >> 16) & 0xFF;
    motor_slot.data[14] = (tor_hex >> 8) & 0xFF;
    motor_slot.data[15] = tor_hex & 0xFF;
}

extern "C" void set_phyarc_SpeedControl(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float spd, float tor) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 12)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x08;
    motor_slot.data[3] = 0x06;

    uint32_t spd_hex = floatToBigEndianHex(spd);
    uint32_t tor_hex = floatToBigEndianHex(tor);

    motor_slot.data[4] = (spd_hex >> 24) & 0xFF;
    motor_slot.data[5] = (spd_hex >> 16) & 0xFF;
    motor_slot.data[6] = (spd_hex >> 8) & 0xFF;
    motor_slot.data[7] = spd_hex & 0xFF;

    motor_slot.data[8] = (tor_hex >> 24) & 0xFF;
    motor_slot.data[9] = (tor_hex >> 16) & 0xFF;
    motor_slot.data[10] = (tor_hex >> 8) & 0xFF;
    motor_slot.data[11] = tor_hex & 0xFF;
}

extern "C" void set_phyarc_TorqueControl(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float tor) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 8)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x09;
    motor_slot.data[3] = 0x06;

    uint32_t tor_hex = floatToBigEndianHex(tor);

    motor_slot.data[4] = (tor_hex >> 24) & 0xFF;
    motor_slot.data[5] = (tor_hex >> 16) & 0xFF;
    motor_slot.data[6] = (tor_hex >> 8) & 0xFF;
    motor_slot.data[7] = tor_hex & 0xFF;
}

extern "C" void set_phyarc_CurrentControl(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float cur) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 8)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x0A;
    motor_slot.data[3] = 0x06;

    uint32_t cur_hex = floatToBigEndianHex(cur);

    motor_slot.data[4] = (cur_hex >> 24) & 0xFF;
    motor_slot.data[5] = (cur_hex >> 16) & 0xFF;
    motor_slot.data[6] = (cur_hex >> 8) & 0xFF;
    motor_slot.data[7] = cur_hex & 0xFF;
}

extern "C" void call_phyarc_getmode(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 4)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x03;
    motor_slot.data[3] = 0x03;
}

extern "C" void call_phyarc_getstate(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 4)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x0D;
    motor_slot.data[3] = 0x03;
}

// 获得逆变器状态，包括mos温度、线圈温度和母线电压
extern "C" void call_phyarc_getInverterstae(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id) {
    if (!preparePhyArcSlot(TxMessage, data_channel, motor_id, 4)) {
        return;
    }

    Motor_Msg &motor_slot = TxMessage->motor[data_channel - 1];
    motor_slot.data[0] = static_cast<uint8_t>(motor_id);
    motor_slot.data[1] = 0x06;
    motor_slot.data[2] = 0x0F;
    motor_slot.data[3] = 0x03;
}

extern "C" void set_phyarc_current(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float cur) {
    set_phyarc_CurrentControl(TxMessage, data_channel, motor_id, cur);
}

extern "C" void set_phyarc_speed(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float spd) {
    set_phyarc_SpeedControl(TxMessage, data_channel, motor_id, spd, 0.0f);
}

extern "C" void set_phyarc_position(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id, float pos) {
    set_phyarc_TorAPosContorl(TxMessage, data_channel, motor_id, pos, 0.0f, 0.0f);
}

extern "C" void set_phyarc_stop(EtherCAT_Msg *TxMessage, uint8_t data_channel, uint32_t motor_id) {
    clearPhyArcSlot(TxMessage, data_channel);
}

void PhyArcHandleInitOnly(const Motor *motor, int slave_idx, int requested_mode, const YKSMotorData *mot_data) {
    (void) slave_idx;
    (void) mot_data;

    if (motor == nullptr) {
        return;
    }

    const uint8_t target_mode = mapPhyArcMode(requested_mode);
    if (target_mode == Mode_Null) {
        return;
    }

    // TODO: 如果 PhyArc 上电后需要固定初始化顺序，在这里调用 set_phyarc_enable / set_phyarc_mode。
}

void PhyArcHandleRuntimeCommand(const Motor *motor, int slave_idx, int index, const YKSMotorData *mot_data) {
    (void) index;

    if (motor == nullptr || mot_data == nullptr) {
        return;
    }

    EtherCAT_Msg *tx_message = nullptr;
    (void) tx_message;
    (void) slave_idx;

    // TODO: 接入现有发送链路后，在这里根据 mot_data[index].mode 分发到
    // set_phyarc_position / set_phyarc_current / set_phyarc_speed / set_phyarc_stop。
}