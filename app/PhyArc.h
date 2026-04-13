#ifndef PHYARC_H
#define PHYARC_H

extern "C" {
#include "motor_control.h"
}

void PhyArcHandleInitOnly(const Motor *motor, int slave_idx, int requested_mode, const YKSMotorData *mot_data);

void PhyArcHandleRuntimeCommand(const Motor *motor, int slave_idx, int index, const YKSMotorData *mot_data);

#endif