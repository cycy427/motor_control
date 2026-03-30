#include "SBusReceiver.h"
#include "JoyStickHandler.h"
#include "Z1Legs.h"
#include "BmsHandler.h"
#include "SocketReceiver.h"
#include "SocketSender.h"
YKSMotorData my_motor_data[Z1_NUM_MOTOR];

void squat_control(const float pos) {
    // my_motor_data[Z1JointIndex::LeftHipYaw].pos_des_ = pos * 0.2; //左右转动
    my_motor_data[Z1JointIndex::LeftHipYaw].pos_des_ = 0; //左右转动
    my_motor_data[Z1JointIndex::LeftHipPitch].pos_des_ = -pos * 0.25; //前后运动
    // my_motor_data[Z1JointIndex::LeftHipRoll].pos_des_ = pos * 0.2;//外摆
    my_motor_data[Z1JointIndex::LeftHipRoll].pos_des_ = 0;
    my_motor_data[Z1JointIndex::LeftKnee].pos_des_ = pos * 0.7;
    my_motor_data[Z1JointIndex::LeftAnkleA].pos_des_ = -pos * 0.1;
    my_motor_data[Z1JointIndex::LeftAnkleB].pos_des_ = pos * 0.1;

    // my_motor_data[Z1JointIndex::RightHipYaw].pos_des_ = pos * 0.2; //左右转动
    my_motor_data[Z1JointIndex::RightHipYaw].pos_des_ = 0; //左右转动
    my_motor_data[Z1JointIndex::RightHipPitch].pos_des_ = pos * 0.25; //ok
    // my_motor_data[Z1JointIndex::RightHipRoll].pos_des_ = -pos * 0.2 - 0.1; //8
    my_motor_data[Z1JointIndex::RightHipRoll].pos_des_ = 0;
    my_motor_data[Z1JointIndex::RightKnee].pos_des_ = -pos * 0.7; //ok
    my_motor_data[Z1JointIndex::RightAnkleA].pos_des_ = pos * 0.1;
    my_motor_data[Z1JointIndex::RightAnkleB].pos_des_ = -pos * 0.1;

    // my_motor_data[Z1JointIndex::WaistYaw].pos_des_ = pos;
}

int main() {
    if (bool Ethernet_Status = CAT_Init("enp5s0"); !Ethernet_Status) { exit(1); } //如果初始化失败，则直接退出程序
    auto joystick_device = "/dev/input/js0";
    // auto battery = "/dev/ttyUSB1";
    const auto joystick_handler = std::make_shared<JoyStickHandler>(joystick_device);
    // const auto battery_handler = std::make_shared<BmsHandler>(battery);
    const SBusReceiver sbus_receiver("/dev/SBUS1");
    Z1Legs z1_legs;
    z1_legs.setJoyStickHandler(joystick_handler);
    // z1_legs.setBatteryHandler(battery_handler);

    SocketReceiver receiver(YKS_PORT);
    SocketSender sender("127.0.0.1",USR_PORT);
    // receiver.startListening();
    // sender.sendDataPeriodically();

    // double pos_pitch = 0;
    // double pos_roll = 0;
    while (true) {
        if (z1_legs.stop_) {
            break;
        }
        // SBusData data = sbus_receiver.getData();
        // SBusReceiver::print_data(data);
        // pos = data.ch[2] / 672.0 * 4;
        // JoystickState state = joystick_handler->getState();
        // JoyStickHandler::print_state(state);
        // pos_roll = state.left_stick_x / 40000.0 * 10;
        // pos_pitch = state.left_stick_y / 40000.0 * 10;
        // printf("pos: %f\n", pos);

        // squat_control(pos);
        // my_motor_data[LeftAnklePitch].pos_des_ = pos_pitch; //设置电机目标位置
        // my_motor_data[LeftAnkleRoll].pos_des_ = pos_roll; //设置电机目标位置
        // my_motor_data[RightAnklePitch].pos_des_ = pos_pitch; //设置电机目标位置
        // my_motor_data[RightAnkleRoll].pos_des_ = pos_roll; //设置电机目标位置
        // my_motor_data[LeftAnklePitch].ff_ = pos_pitch; //设置电机目标位置
        // my_motor_data[LeftAnkleRoll].ff_ = pos_roll; //设置电机目标位置
        // my_motor_data[RightAnklePitch].ff_ = pos_pitch; //设置电机目标位置
        // my_motor_data[RightAnkleRoll].ff_ = pos_roll; //设置电机目标位置
        // my_motor_data[LeftKnee].ff_ = -pos_roll; //设置电机目标位置
        // my_motor_data[RightKnee].ff_ = pos_roll; //设置电机目标位置
        // my_motor_data[Z1JointIndex::LeftAnkleRoll].ff_ = pos_roll; //设置电机目标位置
        receiver.getSocketMotorCMD(my_motor_data);
        z1_legs.setMotorKpKd(my_motor_data); //专门设置电机KP、KD值，调用了这个函数之后就会将原来设置在Z1legs类里面的默认KP、KD值覆盖掉
        z1_legs.setMotorCommand(my_motor_data); //设置电机指令
        z1_legs.getMotorData(my_motor_data); //获取电机数据
        sender.sendSocketMotorData(my_motor_data); //通过Socket反馈电机当前的数据
        // printf("ld_pitch: %f  ld_roll: %f l_pitch: %f  l_roll: %f rd_pitch: %f  rd_roll: %f r_pitch: %f  r_roll: %f \n", pos_pitch, pos_roll,
        //        my_motor_data[LeftAnklePitch].tau_, my_motor_data[LeftAnkleRoll].tau_,pos_pitch, pos_roll,
        //        my_motor_data[RightAnklePitch].tau_, my_motor_data[RightAnkleRoll].tau_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //读取周期为1ms
    }
    runThread.join(); //runThread.join(); 的主要功能是确保 main 函数在退出之前等待 runThread 线程完成其任务。
    // 这样做的目的是为了确保程序在退出前所有的后台任务都得到了正确地执行和清理，避免数据丢失或资源泄露。
    return 0;
}
