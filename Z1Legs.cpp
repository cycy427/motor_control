//
// Created by gold on 25-3-1.
//

#include "Z1Legs.h"
extern std::atomic<bool> stop_thread;

namespace {
double clampUnit(const double value) {
    if (value > 1.0) {
        return 1.0;
    }
    if (value < -1.0) {
        return -1.0;
    }
    return value;
}

double finiteOrZero(const double value) {
    return std::isfinite(value) ? value : 0.0;
}
}

Z1Legs::Z1Legs() : stop_(false), time_(0.0), control_dt_(1), duration_(3.0), counter_(0),
                   mode_pr_(Mode::PR), mode_machine_(0) {
    control_thread_ = std::make_shared<std::thread>(&Z1Legs::Control, this);
    // Assign Kp and Kd values to motorDate_recv
    for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
        motor_data_[i].kp_ = Kp[i];
        motor_data_[i].kd_ = Kd[i];
        // motorDate_recv[i].kp_ = Kp[i];
        // motorDate_recv[i].kd_ = Kd[i];
    }
}

void Z1Legs::PrintFrequency(int &iteration_count, std::chrono::high_resolution_clock::time_point &start_time) {
    const auto current_time = std::chrono::high_resolution_clock::now();
    if (const auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        elapsed_time >= 1) {
        // 每秒打印一次频率
        const double frequency = static_cast<double>(iteration_count) / elapsed_time;
        std::cout << "\tExecution frequency: " << frequency << " Hz" << std::endl;
        iteration_count = 0;
        start_time = current_time;
    }
}

void Z1Legs::PrintMotorState(const int size) const {

    // 打印 flag 状态
    attron(COLOR_PAIR(3));
    mvprintw(2, 2, "IMU: %s", is_imu_run_ ? "OK" : "ERR");
    mvprintw(2, 12, "Hcmd: %s", is_hcmd_run_ ? "OK" : "ERR");
    mvprintw(2, 22, "Logic: %s", is_logic_run_ ? "OK" : "ERR");
    mvprintw(2, 36, "Bms: %s", is_bms_run_ ? "OK" : "ERR");
    attroff(COLOR_PAIR(3));

    attron(COLOR_PAIR(1)); // Blue for position
    mvprintw(4, 0, "Motor ID | ");
    attroff(COLOR_PAIR(1));

    attron(COLOR_PAIR(2)); // Green for velocity
    mvprintw(4, 11, "Pos (pos_) | ");
    attroff(COLOR_PAIR(2));

    attron(COLOR_PAIR(3)); // Red for torque
    mvprintw(4, 23, "Vel (vel_) | ");
    attroff(COLOR_PAIR(3));

    attron(COLOR_PAIR(4)); // Yellow for desired position
    mvprintw(4, 34, "Tau (tau_) | ");
    attroff(COLOR_PAIR(4));

    attron(COLOR_PAIR(5)); // Magenta for desired velocity
    mvprintw(4, 46, "Des Pos (pos_des_) | ");
    attroff(COLOR_PAIR(5));

    attron(COLOR_PAIR(6)); // Cyan for KP, KD, FF
    mvprintw(4, 63, "Des Vel (vel_des_) | ");
    attroff(COLOR_PAIR(6));

    mvprintw(4, 79, "KP (kp_) | ");
    mvprintw(4, 87, "KD (kd_) | ");
    mvprintw(4, 95, "FF (ff_)");
    if (battery_enable_) {
        attron(COLOR_PAIR(3));
        mvprintw(4, 103, "Soc | ");
        mvprintw(4, 111, "Temperature | ");
        BmsState state = battery_handler_->getState();
        mvprintw(6, 103, "%.2d", state.soc);
        mvprintw(6, 111, "%.2f", state.temperature);
        attroff(COLOR_PAIR(3));
    }

    mvprintw(5, 0,
             "------------------------------------------------------------------------------------------------------------------");

    // 打印电机状态
    for (int i = 0; i < size; ++i) {
        move(i + 8, 0);
        clrtoeol();

        attron(COLOR_PAIR(1));
        mvprintw(i + 8, 0, "%d", i);
        attroff(COLOR_PAIR(1));

        attron(COLOR_PAIR(2));
        mvprintw(i + 8, 11, "%9.3f", finiteOrZero(motor_print_[i].pos_));
        attroff(COLOR_PAIR(2));

        attron(COLOR_PAIR(3));
        mvprintw(i + 8, 24, "%9.3f", finiteOrZero(motor_print_[i].vel_));
        attroff(COLOR_PAIR(3));

        attron(COLOR_PAIR(4));
        mvprintw(i + 8, 35, "%9.3f", finiteOrZero(motor_print_[i].tau_));
        attroff(COLOR_PAIR(4));

        attron(COLOR_PAIR(5));
        mvprintw(i + 8, 46, "%9.3f", finiteOrZero(motor_print_[i].pos_des_));
        attroff(COLOR_PAIR(5));

        attron(COLOR_PAIR(6));
        mvprintw(i + 8, 63, "%9.3f", finiteOrZero(motor_print_[i].vel_des_));
        attroff(COLOR_PAIR(6));

        mvprintw(i + 8, 79, "%7.3f", finiteOrZero(motor_print_[i].kp_));
        mvprintw(i + 8, 87, "%7.3f", finiteOrZero(motor_print_[i].kd_));
        mvprintw(i + 8, 95, "%7.3f", finiteOrZero(motor_print_[i].ff_));
    }
}


void Z1Legs::Control() {
#ifdef PRINT_MOTOR_STATE
    initscr(); // Initialize the screen
    cbreak(); // Line buffering disabled
    noecho(); // Don't echo keys
    keypad(stdscr, TRUE); // Enable function keys

    // Start color
    if (has_colors()) {
        start_color();
        init_pair(1, COLOR_BLUE, COLOR_BLACK);
        init_pair(2, COLOR_GREEN, COLOR_BLACK);
        init_pair(3, COLOR_RED, COLOR_BLACK);
        init_pair(4, COLOR_YELLOW, COLOR_BLACK);
        init_pair(5, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(6, COLOR_CYAN, COLOR_BLACK);
    }
#endif
    // float pos = 0;
    // auto start_time = std::chrono::high_resolution_clock::now(); // 记录开始时间
    // int iteration_count = 0;
    // MotorDataLogger motor_data_logger;

    while (!stop_thread.load()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) {
                break;
            }
        }
        {
            //创建的 lock 对象的作用域内（即 {} 包围的代码块），
            //可以安全地对 motor_data_ 进行读写操作，因为此时 mutex_ 已经被锁定，
            //其他线程无法同时修改 motor_data_。注意他只在花括号内有效，
            //离开花括号作用域后，mutex_ 自动解锁。
            // SaveMotorDataToCSV("motor_data_log.csv");  // 每次循环都追加写入
            std::lock_guard lock(mutex_);
            EtherCAT_Send_Command(motor_data_);
            // printf( "MotorData:)");
            // EtherCAT_Send_Command(motorDate_recv);
        }
        updateMotorData();
        std::this_thread::sleep_for(std::chrono::milliseconds(control_dt_)); //控制周期为2ms
        //打印控制循环线程执行频率
        // iteration_count++;
        // PrintFrequency(iteration_count, start_time);
        //打印电机状态
        PrintMotorState(Z1_NUM_MOTOR); //只有头文件中的宏定义PRINT_MOTOR_STATE打开时才会打印电机状态，否则调用无效 13代表有13个电机，对应会产生13行数据
        refresh(); // Refresh the screen
        // motor_data_logger.print_log(motor_data_); //保存电机数据
    }
    endwin(); // End curses mode
}

void Z1Legs::updateMotorData() {
    std::lock_guard lock(mutex_);
    // std::memcpy(motor_data_, motorDate_recv, Z1_NUM_MOTOR * sizeof(YKSMotorData));
    User_Get_Motor_Data(motor_data_); //通过EtherCAT读取电机数据
}

Z1Legs::InverseKinematicsResult Z1Legs::inverse_kinematics(double roll, double pitch) const {
    const double H1 = 330.0, H2 = 120.0;
    const double L1 = 330.0, L2 = 210.0, L3 = 76.0, L4 = 76.0;
    const double xw_c1 = -76.0, yw_c1 = 35.5, zw_c1 = 0.0;
    const double xw_c2 = -76.0, yw_c2 = -35.5, zw_c2 = 0.0;
    const double y_b1 = 35.5, y_b2 = -35.5;

    double cos_p = cos(pitch), sin_p = sin(pitch);
    double sin_r = sin(-roll), cos_r = cos(-roll);

    // 脚踝A的坐标变换
    double x_c1 = cos_p * xw_c1 + sin_r * sin_p * yw_c1 + cos_r * sin_p * zw_c1;
    double y_z1 = cos_r * yw_c1 - sin_r * zw_c1;
    double z_c1 = -sin_p * xw_c1 + sin_r * cos_p * yw_c1 + cos_r * cos_p * zw_c1 - H1;

    // 脚踝B的坐标变换
    double x_c2 = cos_p * xw_c2 + sin_r * sin_p * yw_c2 + cos_r * sin_p * zw_c2;
    double y_z2 = cos_r * yw_c2 - sin_r * zw_c2;
    double z_c2 = -sin_p * xw_c2 + sin_r * cos_p * yw_c2 + cos_r * cos_p * zw_c2 - H1;

    // 计算theta1（脚踝A）
    double k1 = -2 * L3 * x_c1;
    double k2 = -2 * L3 * z_c1;
    double a1 = L3*L3 + x_c1*x_c1 + pow(y_b1 - y_z1, 2) + z_c1*z_c1 - L1*L1;
    double denominator1 = sqrt(k1*k1 + k2*k2);
    double theta1 = (denominator1 > 1e-6) ? asin(clampUnit(a1 / denominator1)) - atan2(k1, k2) : 0.0;

    // 计算theta2（脚踝B）
    double k3 = -2 * L4 * x_c2;
    double k4 = -2 * L4 * (z_c2 + H2);
    double a2 = L4*L4 + x_c2*x_c2 + pow(y_b2 - y_z2, 2) + pow(z_c2 + H2, 2) - L2*L2;
    double denominator2 = sqrt(k3*k3 + k4*k4);
    double theta2 = (denominator2 > 1e-6) ? asin(clampUnit(a2 / denominator2)) - atan2(k3, k4) : 0.0;

    return {theta1, theta2};
}

void Z1Legs::compute_jacobian(double roll, double pitch, double J[2][2], double delta) const {
    // 数值法计算雅可比矩阵 (中心差分法)
    // dtheta/droll
    auto theta_p = inverse_kinematics(roll + delta, pitch);
    auto theta_m = inverse_kinematics(roll - delta, pitch);
    J[0][0] = (theta_p.theta1 - theta_m.theta1) / (2 * delta); // dtheta1/droll
    J[1][0] = (theta_p.theta2 - theta_m.theta2) / (2 * delta); // dtheta2/droll

    // dtheta/dpitch
    theta_p = inverse_kinematics(roll, pitch + delta);
    theta_m = inverse_kinematics(roll, pitch - delta);
    J[0][1] = (theta_p.theta1 - theta_m.theta1) / (2 * delta); // dtheta1/dpitch
    J[1][1] = (theta_p.theta2 - theta_m.theta2) / (2 * delta); // dtheta2/dpitch
}

void Z1Legs::inverse_velocity(double roll, double pitch, const double end_vel[2], double joint_vel[2]) const {
    double J[2][2];
    compute_jacobian(roll, pitch, J);

    // 计算雅可比矩阵的逆
    double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (fabs(det) < 1e-6) {
        joint_vel[0] = joint_vel[1] = 0.0;
        return;
    }

    double invJ[2][2] = {
        { J[1][1]/det, -J[0][1]/det },
        { -J[1][0]/det, J[0][0]/det }
    };

    // 计算关节速度
    joint_vel[0] = invJ[0][0] * end_vel[0] + invJ[0][1] * end_vel[1]; // dtheta1/dt
    joint_vel[1] = invJ[1][0] * end_vel[0] + invJ[1][1] * end_vel[1]; // dtheta2/dt
}

void Z1Legs::forward_kinematics(double theta1, double theta2, double& roll, double& pitch) const {
    // 牛顿迭代法求解正向运动学
    if (!std::isfinite(theta1) || !std::isfinite(theta2)) {
        roll = 0.0;
        pitch = 0.0;
        return;
    }

    double current_roll = 0.0, current_pitch = 0.0;
    const int max_iter = 100;
    const double tol = 1e-4;

    for (int i = 0; i < max_iter; ++i) {
        auto current_theta = inverse_kinematics(current_roll, current_pitch);
        double res1 = current_theta.theta1 - theta1;
        double res2 = current_theta.theta2 - theta2;
        if (!std::isfinite(res1) || !std::isfinite(res2)) {
            break;
        }
        if (hypot(res1, res2) < tol) break;

        // 计算雅可比矩阵
        double J[2][2];
        compute_jacobian(current_roll, current_pitch, J);
        double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
        if (fabs(det) < 1e-6) break;

        // 更新roll和pitch
        double delta_roll = (-res1 * J[1][1] + res2 * J[0][1]) / det;
        double delta_pitch = (res1 * J[1][0] - res2 * J[0][0]) / det;
        if (!std::isfinite(delta_roll) || !std::isfinite(delta_pitch)) {
            break;
        }
        current_roll += delta_roll;
        current_pitch += delta_pitch;
        if (!std::isfinite(current_roll) || !std::isfinite(current_pitch)) {
            current_roll = 0.0;
            current_pitch = 0.0;
            break;
        }
    }

    roll = current_roll;
    pitch = current_pitch;
}

void Z1Legs::pseudo_inverse(const double J[2][2], double invJ[2][2]) const {
    double det = J[0][0]*J[1][1] - J[0][1]*J[1][0];
    if(fabs(det) > 1e-6) { // 可逆情况
        invJ[0][0] =  J[1][1]/det;
        invJ[0][1] = -J[0][1]/det;
        invJ[1][0] = -J[1][0]/det;
        invJ[1][1] =  J[0][0]/det;
    } else { // 伪逆处理
        double svd[4];
        svd[0] = J[0][0]*J[0][0] + J[1][0]*J[1][0];
        svd[1] = J[0][0]*J[0][1] + J[1][0]*J[1][1];
        svd[2] = J[0][1]*J[0][1] + J[1][1]*J[1][1];
        double lambda = 1e-6;
        invJ[0][0] = (svd[2] + lambda)*J[0][0] - svd[1]*J[0][1];
        invJ[1][0] = (svd[0] + lambda)*J[0][1] - svd[1]*J[0][0];
        invJ[0][1] = (svd[2] + lambda)*J[1][0] - svd[1]*J[1][1];
        invJ[1][1] = (svd[0] + lambda)*J[1][1] - svd[1]*J[1][0];
        double inv_norm = 1.0/( (svd[0]+lambda)*(svd[2]+lambda) - svd[1]*svd[1] );
        invJ[0][0] *= inv_norm;
        invJ[1][0] *= inv_norm;
        invJ[0][1] *= inv_norm;
        invJ[1][1] *= inv_norm;
    }
}

YKSMotorData
Z1Legs::AnkleA_inverse_kinematics(const YKSMotorData &pitch_joint_cmd, const YKSMotorData &roll_joint_cmd,
                                  int direction) {
    int Pitch, Roll;
    if (direction == LeftLeg) {
        Pitch = 4;
        Roll = 5;
    } else {
        Pitch = 10;
        Roll = 11;
    }

    YKSMotorData cmd;

    // 位置逆解
	const double desired_roll = roll_joint_cmd.pos_des_;
    const double desired_pitch = pitch_joint_cmd.pos_des_;
    auto theta = inverse_kinematics(desired_roll, desired_pitch);
    cmd.pos_des_ = theta.theta1;

    // 速度逆解
    const double end_vel[2] = {roll_joint_cmd.vel_des_, pitch_joint_cmd.vel_des_};
    double joint_vel[2];
    inverse_velocity(desired_roll, desired_pitch, end_vel, joint_vel);
    cmd.vel_des_ = joint_vel[0];

    // 前馈力矩计算（考虑雅可比转置）
    cmd.ff_ = (pitch_joint_cmd.ff_ - roll_joint_cmd.ff_) * PR_directionMotor_[0] / 2.0;
    return cmd;
}

YKSMotorData
Z1Legs::AnkleB_inverse_kinematics(const YKSMotorData &pitch_joint_cmd, const YKSMotorData &roll_joint_cmd,
                                  int direction) {
    int Pitch, Roll;
    if (direction == LeftLeg) {
        Pitch = 4;
        Roll = 5;
    } else {
        Pitch = 10;
        Roll = 11;
    }
    YKSMotorData cmd;

    // 位置逆解
    const double desired_roll = roll_joint_cmd.pos_des_;
    const double desired_pitch = pitch_joint_cmd.pos_des_;
    auto theta = inverse_kinematics(desired_roll, desired_pitch);
    cmd.pos_des_ = theta.theta2;

    // 速度逆解
    const double end_vel[2] = {roll_joint_cmd.vel_des_, pitch_joint_cmd.vel_des_};
    double joint_vel[2];
    inverse_velocity(desired_roll, desired_pitch, end_vel, joint_vel);
    cmd.vel_des_ = joint_vel[1];

    // 前馈力矩计算
    cmd.ff_ = (pitch_joint_cmd.ff_ + roll_joint_cmd.ff_) * PR_directionMotor_[1] / 2.0;
    return cmd;
}

YKSMotorData Z1Legs::Pitch_forward_kinematics(const YKSMotorData &Ankle_A_motors,
                                              const YKSMotorData &Ankle_B_motors, int direction) {
    int Pitch, Roll;
    if (direction == LeftLeg) {
        Pitch = 4;
        Roll = 5;
    } else {
        Pitch = 10;
        Roll = 11;
    }
  YKSMotorData data;
    // 位置正解
    const double theta1 = Ankle_A_motors.pos_* LegDirectionMotor_[Roll];
    const double theta2 = Ankle_B_motors.pos_* LegDirectionMotor_[Pitch];
    double roll, pitch;
    forward_kinematics(theta1, theta2, roll, pitch);
    data.pos_ = pitch;

    // 速度正解
    double J[2][2];
    compute_jacobian(roll, pitch, J);
    const double dtheta1 = Ankle_A_motors.vel_* LegDirectionMotor_[Roll];
    const double dtheta2 = Ankle_B_motors.vel_* LegDirectionMotor_[Pitch];
    data.vel_ = J[1][0] * dtheta1 + J[1][1] * dtheta2;

    // 力矩正解（τ_joint = J^T * τ_motor）
    const double tau1 = Ankle_A_motors.tau_* LegDirectionMotor_[Roll];
    const double tau2 = Ankle_B_motors.tau_* LegDirectionMotor_[Pitch];
    data.tau_ = J[0][1] * tau1 + J[1][1] * tau2; // Pitch对应雅可比第二行

    return data;
}

YKSMotorData Z1Legs::Roll_forward_kinematics(const YKSMotorData &Ankle_A_motors,
                                             const YKSMotorData &Ankle_B_motors, int direction) {
    int Pitch, Roll;
    if (direction == LeftLeg) {
        Pitch = 4;
        Roll = 5;
    } else {
        Pitch = 10;
        Roll = 11;
    }
  YKSMotorData data;
    // 位置正解
    const double theta1 = Ankle_A_motors.pos_* LegDirectionMotor_[Roll];
    const double theta2 = Ankle_B_motors.pos_* LegDirectionMotor_[Pitch];
    double roll, pitch;
    forward_kinematics(theta1, theta2, roll, pitch);
    data.pos_ = roll;

    // 速度正解
    double J[2][2];
    compute_jacobian(roll, pitch, J);
    const double dtheta1 = Ankle_A_motors.vel_* LegDirectionMotor_[Roll];
    const double dtheta2 = Ankle_B_motors.vel_* LegDirectionMotor_[Pitch];
    data.vel_ = J[0][0] * dtheta1 + J[0][1] * dtheta2; // Roll对应雅可比第一行

    // 力矩正解
    const double tau1 = Ankle_A_motors.tau_* LegDirectionMotor_[Roll];
    const double tau2 = Ankle_B_motors.tau_* LegDirectionMotor_[Pitch];
    data.tau_ = J[0][0] * tau1 + J[1][0] * tau2;

    return data;
}

//以上都是私有函数，不能从外部调用
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//以下都是公有函数，可以从外部调用

Z1Legs::~Z1Legs() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "Z1Legs Destructor "  << std::endl;\
    stop_ = true;
    if (control_thread_->joinable()) {
        control_thread_->join();
    }
}

void Z1Legs::setJoyStickHandler(const std::shared_ptr<JoyStickHandler> &handler) {
    joy_stick_handler_ = handler;
    joystick_enable_ = true;
}

void Z1Legs::setBatteryHandler(const std::shared_ptr<BmsHandler> &handler) {
    battery_handler_ = handler;
    battery_enable_ = true;
}

//获取IMU的状态
void Z1Legs::getIMUFlag(bool flag) {
    std::lock_guard lock(mutex_);
    is_imu_run_ = flag;
}

//获取Hcmd的状态
void Z1Legs::getHcmdFlag(bool flag) {
    std::lock_guard lock(mutex_);
    is_hcmd_run_ = flag;
}

//获取Logic的状态
void Z1Legs::getLogicFlag(bool flag) {
    std::lock_guard lock(mutex_);
    is_logic_run_ = flag;
}

//获取Bms的状态
void Z1Legs::getBmsFlag(bool flag) {
    std::lock_guard lock(mutex_);
    is_bms_run_ = flag;
}

void Z1Legs::setMotorCommand(const YKSMotorData *data) {
    // 检查输入数据指针是否为空
    if (data == nullptr) {
        throw std::invalid_argument("Input data pointer cannot be null");
    }
    std::lock_guard lock(mutex_);
    if (mode_pr_ == Mode::PR) {
        YKSMotorData tempData;
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            motor_data_[i].mode = data[i].mode;
            motor_print_[i].mode = data[i].mode;
            if (i == LeftAnkleA) {
                tempData = AnkleA_inverse_kinematics(data[LeftAnklePitch], data[LeftAnkleRoll], LeftLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_* LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_* LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_* LegDirectionMotor_[i];
                motor_print_[i].pos_des_ = data[i].pos_des_;
                motor_print_[i].vel_des_ = data[i].vel_des_;
                motor_print_[i].ff_ = data[i].ff_;
            } else if (i == LeftAnkleB) {
                tempData = AnkleB_inverse_kinematics(data[LeftAnklePitch], data[LeftAnkleRoll], LeftLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_* LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_* LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_* LegDirectionMotor_[i];
                motor_print_[i].pos_des_ = data[i].pos_des_;
                motor_print_[i].vel_des_ = data[i].vel_des_;
                motor_print_[i].ff_ = data[i].ff_;
            } else if (i == RightAnkleA) {
                tempData = AnkleA_inverse_kinematics(data[RightAnklePitch], data[RightAnkleRoll], RightLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_* LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_* LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_* LegDirectionMotor_[i];
                motor_print_[i].pos_des_ = data[i].pos_des_;
                motor_print_[i].vel_des_ = data[i].vel_des_;
                motor_print_[i].ff_ = data[i].ff_;
            } else if (i == RightAnkleB) {
                tempData = AnkleB_inverse_kinematics(data[RightAnklePitch], data[RightAnkleRoll], RightLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_* LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_* LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_* LegDirectionMotor_[i];
                motor_print_[i].pos_des_ = data[i].pos_des_;
                motor_print_[i].vel_des_ = data[i].vel_des_;
                motor_print_[i].ff_ = data[i].ff_;
            } else {
                motor_data_[i].pos_des_ = data[i].pos_des_ * LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = data[i].vel_des_ * LegDirectionMotor_[i];
                motor_data_[i].ff_ = data[i].ff_ * LegDirectionMotor_[i];
                motor_print_[i].pos_des_ = data[i].pos_des_;
                motor_print_[i].vel_des_ = data[i].vel_des_;
                motor_print_[i].ff_ = data[i].ff_;
            }
        }
    } else {
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            motor_data_[i].mode = data[i].mode;
            motor_data_[i].pos_des_ = data[i].pos_des_ * LegDirectionMotor_[i];
            motor_data_[i].vel_des_ = data[i].vel_des_ * LegDirectionMotor_[i];
            motor_data_[i].ff_ = data[i].ff_ * LegDirectionMotor_[i];
            motor_print_[i].mode = data[i].mode;
            motor_print_[i].pos_des_ = data[i].pos_des_;
            motor_print_[i].vel_des_ = data[i].vel_des_;
            motor_print_[i].ff_ = data[i].ff_;
        }
    }
}


void Z1Legs::setMotorKpKd(const YKSMotorData *data) {
    std::lock_guard lock(mutex_);
    // std::memcpy(motor_data_, data, Z1_NUM_MOTOR * sizeof(YKSMotorData));
    for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
        motor_data_[i].kp_ = data[i].kp_;
        motor_data_[i].kd_ = data[i].kd_;
        motor_print_[i].kp_ = data[i].kp_;
        motor_print_[i].kd_ = data[i].kd_;
    }
}

void Z1Legs::getMotorData(YKSMotorData *data) {
    std::lock_guard lock(mutex_);
    // std::memcpy(data, motor_data_, Z1_NUM_MOTOR * sizeof(YKSMotorData));
    if (mode_pr_ == Mode::PR) {
        YKSMotorData temp_data;
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            if (i == LeftAnkleA) {
                temp_data = Pitch_forward_kinematics(motor_data_[LeftAnkleA], motor_data_[LeftAnkleB], LeftLeg);
                data[LeftAnklePitch].pos_ = temp_data.pos_;
                data[LeftAnklePitch].vel_ = temp_data.vel_;
                data[LeftAnklePitch].tau_ = temp_data.tau_;
                motor_print_[LeftAnklePitch].pos_ = temp_data.pos_;
                motor_print_[LeftAnklePitch].vel_ = temp_data.vel_;
                motor_print_[LeftAnklePitch].tau_ = temp_data.tau_;

            } else if (i == LeftAnkleB) {
                temp_data = Roll_forward_kinematics(motor_data_[LeftAnkleA], motor_data_[LeftAnkleB], LeftLeg);
                data[LeftAnkleRoll].pos_ = temp_data.pos_;
                data[LeftAnkleRoll].vel_ = temp_data.vel_;
                data[LeftAnkleRoll].tau_ = temp_data.tau_;
                motor_print_[LeftAnkleRoll].pos_ = temp_data.pos_;
                motor_print_[LeftAnkleRoll].vel_ = temp_data.vel_;
                motor_print_[LeftAnkleRoll].tau_ = temp_data.tau_;
            } else if (i == RightAnkleA) {
                temp_data = Pitch_forward_kinematics(motor_data_[RightAnkleA], motor_data_[RightAnkleB], RightLeg);
                data[RightAnklePitch].pos_ = temp_data.pos_;
                data[RightAnklePitch].vel_ = temp_data.vel_;
                data[RightAnklePitch].tau_ = temp_data.tau_;
                motor_print_[RightAnklePitch].pos_ = temp_data.pos_;
                motor_print_[RightAnklePitch].vel_ = temp_data.vel_;
                motor_print_[RightAnklePitch].tau_ = temp_data.tau_;
            } else if (i == RightAnkleB) {
                temp_data = Roll_forward_kinematics(motor_data_[RightAnkleA], motor_data_[RightAnkleB], RightLeg);
                data[RightAnkleRoll].pos_ = temp_data.pos_;
                data[RightAnkleRoll].vel_ = temp_data.vel_;
                data[RightAnkleRoll].tau_ = temp_data.tau_;
                motor_print_[RightAnkleRoll].pos_ = temp_data.pos_;
                motor_print_[RightAnkleRoll].vel_ = temp_data.vel_;
                motor_print_[RightAnkleRoll].tau_ = temp_data.tau_;
            } else {
                data[i].pos_ = motor_data_[i].pos_ * LegDirectionMotor_[i];
                data[i].vel_ = motor_data_[i].vel_ * LegDirectionMotor_[i];
                data[i].tau_ = motor_data_[i].tau_ * LegDirectionMotor_[i];
                motor_print_[i].pos_ = motor_data_[i].pos_ * LegDirectionMotor_[i];
                motor_print_[i].vel_ = motor_data_[i].vel_ * LegDirectionMotor_[i];
                motor_print_[i].tau_ = motor_data_[i].tau_ * LegDirectionMotor_[i];
            }
            data[i].error_ = motor_data_[i].error_;
            data[i].mos_temperature_ = motor_data_[i].mos_temperature_;
            data[i].temperature_ = motor_data_[i].temperature_;
            motor_print_[i].error_ = motor_data_[i].error_;
            motor_print_[i].mos_temperature_ = motor_data_[i].mos_temperature_;
            motor_print_[i].temperature_ = motor_data_[i].temperature_;

        }
    } else {
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            data[i].pos_ = motor_data_[i].pos_ * LegDirectionMotor_[i];
            data[i].vel_ = motor_data_[i].vel_ * LegDirectionMotor_[i];
            data[i].tau_ = motor_data_[i].tau_ * LegDirectionMotor_[i];
            data[i].error_ = motor_data_[i].error_;
            data[i].mos_temperature_ = motor_data_[i].mos_temperature_;
            data[i].temperature_ = motor_data_[i].temperature_;
            motor_print_[i].pos_ = motor_data_[i].pos_ * LegDirectionMotor_[i];
            motor_print_[i].vel_ = motor_data_[i].vel_ * LegDirectionMotor_[i];
            motor_print_[i].tau_ = motor_data_[i].tau_ * LegDirectionMotor_[i];
            motor_print_[i].error_ = motor_data_[i].error_;
            motor_print_[i].mos_temperature_ = motor_data_[i].mos_temperature_;
            motor_print_[i].temperature_ = motor_data_[i].temperature_;
        }
    }
}

void Z1Legs::SaveMotorDataToCSV(const std::string &filename) const {
    std::ofstream file(filename, std::ios_base::app); // 使用 app 模式进行追加
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // 如果是第一次写入，添加表头
    if (file.tellp() == 0) {
        file << "ID,pos_,vel_,tau_,pos_des_,vel_des_,kp_,kd_,ff_,mode,error_,temperature_,mos_temperature_\n";
    }
    std::lock_guard lock(mutex_);
    // 遍历 motor_data_ 并写入数据
    for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
        const auto &data = motor_data_[i];
        file << i << ","
                << data.pos_ << ","
                << data.vel_ << ","
                << data.tau_ << ","
                << data.pos_des_ << ","
                << data.vel_des_ << ","
                << data.kp_ << ","
                << data.kd_ << ","
                << data.ff_ << ","
                << static_cast<int>(data.mode) << ","
                << data.error_ << ","
                << data.temperature_ << ","
                << data.mos_temperature_ << "\n";
    }

    file.close();
}
