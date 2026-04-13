#ifndef EYOU_CONTROL_H
#define EYOU_CONTROL_H

extern "C" {
#include "motor_control.h"
}

void EyouHandleInitOnly(const Motor *motor, int slave_idx, int requested_mode, const YKSMotorData *mot_data);

void EyouHandleRuntimeCommand(const Motor *motor, int slave_idx, int index, const YKSMotorData *mot_data);

#endif