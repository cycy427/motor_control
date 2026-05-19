#ifndef ROBOT_LAYOUT_H
#define ROBOT_LAYOUT_H

#define LEG_MOTOR_NUMBER 12
#define BODY_MOTOR_NUMBER 3
#define ARM_MOTOR_NUMBER 14
#define ACTIVE_MOTOR_NUMBER (LEG_MOTOR_NUMBER + BODY_MOTOR_NUMBER + ARM_MOTOR_NUMBER)

#ifndef TOTAL_MOTOR_NUMBER
#define TOTAL_MOTOR_NUMBER 48
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern const int kLegMotorIds[LEG_MOTOR_NUMBER];
extern const int kBodyMotorIds[BODY_MOTOR_NUMBER];
extern const int kArmMotorIds[ARM_MOTOR_NUMBER];
extern const int kAllActiveIds[ACTIVE_MOTOR_NUMBER];
extern const char *kJointNames[TOTAL_MOTOR_NUMBER];

int leg_local_to_global(int local);
int body_local_to_global(int local);
int arm_local_to_global(int local);

int leg_global_to_local(int global_id);
int body_global_to_local(int global_id);
int arm_global_to_local(int global_id);

#ifdef __cplusplus
}
#endif

#endif
