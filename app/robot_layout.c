#include "robot_layout.h"

const int kLegMotorIds[LEG_MOTOR_NUMBER] = {
    0, 1, 2, 3, 4, 5,
    8, 9, 10, 11, 12, 13
};

const int kBodyMotorIds[BODY_MOTOR_NUMBER] = {
    16, 17, 18
};

const int kArmMotorIds[ARM_MOTOR_NUMBER] = {
    24, 25, 26, 27, 28, 29, 30,
    32, 33, 34, 35, 36, 37, 38
};

const int kAllActiveIds[ACTIVE_MOTOR_NUMBER] = {
    0, 1, 2, 3, 4, 5,
    8, 9, 10, 11, 12, 13,
    16, 17, 18,
    24, 25, 26, 27, 28, 29, 30,
    32, 33, 34, 35, 36, 37, 38
};

const char *kJointNames[TOTAL_MOTOR_NUMBER] = {
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "", "",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "", "",
    "WaistRoll", "WaistPitch", "WaistYaw",
    "", "", "", "", "",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftForearmRoll",
    "LeftWristYaw", "LeftWristPitch",
    "",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightForearmRoll",
    "RightWristYaw", "RightWristPitch",
    "", "", "", "", "", "", "", "", ""
};

static int local_to_global(const int *ids, int count, int local) {
    if (local < 0 || local >= count) {
        return -1;
    }
    return ids[local];
}

static int global_to_local(const int *ids, int count, int global_id) {
    for (int i = 0; i < count; ++i) {
        if (ids[i] == global_id) {
            return i;
        }
    }
    return -1;
}

int leg_local_to_global(int local) {
    return local_to_global(kLegMotorIds, LEG_MOTOR_NUMBER, local);
}

int body_local_to_global(int local) {
    return local_to_global(kBodyMotorIds, BODY_MOTOR_NUMBER, local);
}

int arm_local_to_global(int local) {
    return local_to_global(kArmMotorIds, ARM_MOTOR_NUMBER, local);
}

int leg_global_to_local(int global_id) {
    return global_to_local(kLegMotorIds, LEG_MOTOR_NUMBER, global_id);
}

int body_global_to_local(int global_id) {
    return global_to_local(kBodyMotorIds, BODY_MOTOR_NUMBER, global_id);
}

int arm_global_to_local(int global_id) {
    return global_to_local(kArmMotorIds, ARM_MOTOR_NUMBER, global_id);
}
