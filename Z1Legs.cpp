//
// Created by gold on 25-3-1.
//

#include "Z1Legs.h"

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
    attron(COLOR_PAIR(1)); // Blue for position
    mvprintw(2, 0, "Motor ID | ");
    attroff(COLOR_PAIR(1));

    attron(COLOR_PAIR(2)); // Green for velocity
    mvprintw(2, 11, "Pos (pos_) | ");
    attroff(COLOR_PAIR(2));

    attron(COLOR_PAIR(3)); // Red for torque
    mvprintw(2, 23, "Vel (vel_) | ");
    attroff(COLOR_PAIR(3));

    attron(COLOR_PAIR(4)); // Yellow for desired position
    mvprintw(2, 34, "Tau (tau_) | ");
    attroff(COLOR_PAIR(4));

    attron(COLOR_PAIR(5)); // Magenta for desired velocity
    mvprintw(2, 46, "Des Pos (pos_des_) | ");
    attroff(COLOR_PAIR(5));

    attron(COLOR_PAIR(6)); // Cyan for KP, KD, FF
    mvprintw(2, 63, "Des Vel (vel_des_) | ");
    attroff(COLOR_PAIR(6));

    mvprintw(2, 79, "KP (kp_) | ");
    mvprintw(2, 87, "KD (kd_) | ");
    mvprintw(2, 95, "FF (ff_)");
    if (battery_enable_) {
        attron(COLOR_PAIR(3));
        mvprintw(2, 103, "Soc | ");
        mvprintw(2, 111, "Temperature | ");
        BmsState state = battery_handler_->getState();
        mvprintw(4, 103, "%.2f", state.soc);
        mvprintw(4, 111, "%.2f", state.temperature);
        attroff(COLOR_PAIR(3));
    }

    mvprintw(3, 0,
             "------------------------------------------------------------------------------------------------------------------");

    for (int i = 0; i < size; ++i) {
        attron(COLOR_PAIR(1));
        mvprintw(i + 4, 0, "%d", i );
        attroff(COLOR_PAIR(1));

        attron(COLOR_PAIR(2));
        mvprintw(i + 4, 11, "%.3f", motor_data_[i].pos_);
        attroff(COLOR_PAIR(2));

        attron(COLOR_PAIR(3));
        mvprintw(i + 4, 24, "%.3f", motor_data_[i].vel_);
        attroff(COLOR_PAIR(3));

        attron(COLOR_PAIR(4));
        mvprintw(i + 4, 35, "%.3f", motor_data_[i].tau_);
        attroff(COLOR_PAIR(4));

        attron(COLOR_PAIR(5));
        mvprintw(i + 4, 46, "%.3f", motor_data_[i].pos_des_);
        attroff(COLOR_PAIR(5));

        attron(COLOR_PAIR(6));
        mvprintw(i + 4, 63, "%.3f", motor_data_[i].vel_des_);
        attroff(COLOR_PAIR(6));

        mvprintw(i + 4, 79, "%.3f", motor_data_[i].kp_);
        mvprintw(i + 4, 87, "%.3f", motor_data_[i].kd_);
        mvprintw(i + 4, 95, "%.3f", motor_data_[i].ff_);
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
    float pos = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // 记录开始时间
    int iteration_count = 0;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_) {
                break;
            }
        } {
            //创建的 lock 对象的作用域内（即 {} 包围的代码块），
            //可以安全地对 motor_data_ 进行读写操作，因为此时 mutex_ 已经被锁定，
            //其他线程无法同时修改 motor_data_。注意他只在花括号内有效，
            //离开花括号作用域后，mutex_ 自动解锁。
            std::lock_guard lock(mutex_);
            EtherCAT_Send_Command(motor_data_);
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
    }
    endwin(); // End curses mode
}

void Z1Legs::updateMotorData() {
    std::lock_guard lock(mutex_);
    // std::memcpy(motor_data_, motorDate_recv, Z1_NUM_MOTOR * sizeof(YKSMotorData));
    User_Get_Motor_Data(motor_data_); //通过EtherCAT读取电机数据
}

YKSMotorData
Z1Legs::AnkleA_inverse_kinematics(const YKSMotorData &pitch_joint_cmd, const YKSMotorData &roll_joint_cmd) {
    // 逆解计算
    YKSMotorData AnkleA_cmd = {};
    const double theta1_des = pitch_joint_cmd.pos_des_ - roll_joint_cmd.pos_des_;
    const double theta2_des = pitch_joint_cmd.pos_des_ + roll_joint_cmd.pos_des_;

    // 考虑减速比的电机指令
    AnkleA_cmd.pos_des_ = theta1_des * PR_directionMotor_[0];

    // 速度逆解
    const double d_theta1_des = pitch_joint_cmd.vel_des_ - roll_joint_cmd.vel_des_;
    const double d_theta2_des = pitch_joint_cmd.vel_des_ + roll_joint_cmd.vel_des_;
    AnkleA_cmd.vel_des_ = d_theta1_des * PR_directionMotor_[0];

    // 前馈力矩逆解（考虑雅可比矩阵和减速比）
    AnkleA_cmd.ff_ = (pitch_joint_cmd.ff_ - roll_joint_cmd.ff_) * PR_directionMotor_[0] / 2.0;
    return AnkleA_cmd;
}

YKSMotorData
Z1Legs::AnkleB_inverse_kinematics(const YKSMotorData &pitch_joint_cmd, const YKSMotorData &roll_joint_cmd) {
    // 逆解计算
    YKSMotorData AnkleB_cmd = {};
    const double theta1_des = pitch_joint_cmd.pos_des_ - roll_joint_cmd.pos_des_;
    const double theta2_des = pitch_joint_cmd.pos_des_ + roll_joint_cmd.pos_des_;

    // 考虑减速比的电机指令
    AnkleB_cmd.pos_des_ = theta2_des * PR_directionMotor_[1];

    // 速度逆解
    const double d_theta1_des = pitch_joint_cmd.vel_des_ - roll_joint_cmd.vel_des_;
    const double d_theta2_des = pitch_joint_cmd.vel_des_ + roll_joint_cmd.vel_des_;
    AnkleB_cmd.vel_des_ = d_theta2_des * PR_directionMotor_[1];

    // 前馈力矩逆解（考虑雅可比矩阵和减速比）
    AnkleB_cmd.ff_ = (pitch_joint_cmd.ff_ + roll_joint_cmd.ff_) * PR_directionMotor_[1] / 2.0;
    return AnkleB_cmd;
}

YKSMotorData Z1Legs::Pitch_forward_kinematics(const YKSMotorData &Ankle_A_motors,
                                              const YKSMotorData &Ankle_B_motors) {
    YKSMotorData pitch_joint_data = {};

    const double theta1 = Ankle_A_motors.pos_ * PR_directionMotor_[0];
    const double theta2 = Ankle_B_motors.pos_ * PR_directionMotor_[1];

    // 平行四边形机构正解（基于几何关系）
    pitch_joint_data.pos_ = (theta1 + theta2) / 2.0;
    // PR_joint_data[1].pos_des_  = (theta1 - theta2) / 2.0;

    // 速度映射（考虑减速比）
    const double d_theta1 = Ankle_A_motors.vel_ * PR_directionMotor_[0];
    const double d_theta2 = Ankle_B_motors.vel_ * PR_directionMotor_[1];
    pitch_joint_data.vel_ = (d_theta1 + d_theta2) / 2.0;
    // PR_joint_data[1].vel_des_  = (d_theta1 - d_theta2) / 2.0;

    // 力矩映射（考虑雅可比转置和减速比）
    pitch_joint_data.tau_ = Ankle_A_motors.tau_ * PR_directionMotor_[0] + Ankle_B_motors.tau_ * PR_directionMotor_[1];
    // PR_joint_data[1].ff_  = (Ankle_motors[0].tau_ - Ankle_motors[1].tau_) * PR_directionMotor_[1] / 2.0;


    return pitch_joint_data;
}

YKSMotorData Z1Legs::Roll_forward_kinematics(const YKSMotorData &Ankle_A_motors,
                                             const YKSMotorData &Ankle_B_motors)  {
    YKSMotorData pitch_joint_data = {};
    const double theta1 = Ankle_A_motors.pos_ * PR_directionMotor_[0];
    const double theta2 = Ankle_B_motors.pos_ * PR_directionMotor_[1];

    // 平行四边形机构正解（基于几何关系）
    pitch_joint_data.pos_ = (theta1 + theta2) / 2.0;

    // 速度映射（考虑减速比）
    const double d_theta1 = Ankle_A_motors.vel_ * PR_directionMotor_[0];
    const double d_theta2 = Ankle_B_motors.vel_ * PR_directionMotor_[1];
    pitch_joint_data.vel_ = (d_theta1 + d_theta2) / 2.0;

    // 力矩映射（考虑雅可比转置和减速比）
    pitch_joint_data.tau_ = Ankle_A_motors.tau_ * PR_directionMotor_[0] + Ankle_B_motors.tau_ * PR_directionMotor_[1];

    return pitch_joint_data;
}

//以上都是私有函数，不能从外部调用
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//以下都是公有函数，可以从外部调用

Z1Legs::~Z1Legs() {
    std::lock_guard<std::mutex> lock(mutex_);
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
            if (i == LeftAnkleA) {
                tempData = AnkleA_inverse_kinematics(data[LeftAnklePitch], data[LeftAnkleRoll]);
                motor_data_[i].pos_des_ = tempData.pos_des_ * LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_ * LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_ * LegDirectionMotor_[i];
            } else if (i == LeftAnkleB) {
                tempData = AnkleB_inverse_kinematics(data[LeftAnklePitch], data[LeftAnkleRoll]);
                motor_data_[i].pos_des_ = tempData.pos_des_ * LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_ * LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_ * LegDirectionMotor_[i];
            } else if (i == RightAnkleA) {
                tempData = AnkleA_inverse_kinematics(data[RightAnklePitch], data[RightAnkleRoll]);
                motor_data_[i].pos_des_ = tempData.pos_des_ * LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_ * LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_ * LegDirectionMotor_[i];
            } else if (i == RightAnkleB) {
                tempData = AnkleB_inverse_kinematics(data[RightAnklePitch], data[RightAnkleRoll]);
                motor_data_[i].pos_des_ = tempData.pos_des_ * LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = tempData.vel_des_ * LegDirectionMotor_[i];
                motor_data_[i].ff_ = tempData.ff_ * LegDirectionMotor_[i];
            } else {
                motor_data_[i].pos_des_ = data[i].pos_des_ * LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = data[i].vel_des_ * LegDirectionMotor_[i];
                motor_data_[i].ff_ = data[i].ff_ * LegDirectionMotor_[i];
            }
        }
    } else {
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            motor_data_[i].mode = data[i].mode;
            motor_data_[i].pos_des_ = data[i].pos_des_ * LegDirectionMotor_[i];
            motor_data_[i].vel_des_ = data[i].vel_des_ * LegDirectionMotor_[i];
            motor_data_[i].ff_ = data[i].ff_ * LegDirectionMotor_[i];
        }
    }
}


void Z1Legs::setMotorKpKd(const YKSMotorData *data) {
    std::lock_guard lock(mutex_);
    // std::memcpy(motor_data_, data, Z1_NUM_MOTOR * sizeof(YKSMotorData));
    for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
        motor_data_[i].kp_ = data[i].kp_;
        motor_data_[i].kd_ = data[i].kd_;
    }
}

void Z1Legs::getMotorData(YKSMotorData *data) {
    std::lock_guard lock(mutex_);
    // std::memcpy(data, motor_data_, Z1_NUM_MOTOR * sizeof(YKSMotorData));
    if (mode_pr_ == Mode::PR) {
        YKSMotorData temp_data;
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            if (i == LeftAnkleA) {
                temp_data = Pitch_forward_kinematics(motor_data_[LeftAnkleA], motor_data_[LeftAnkleB]);
                data[LeftAnklePitch].pos_ = temp_data.pos_ * LegDirectionMotor_[i];
                data[LeftAnklePitch].vel_ = temp_data.vel_ * LegDirectionMotor_[i];
                data[LeftAnklePitch].tau_ = temp_data.tau_ * LegDirectionMotor_[i];
            } else if (i == LeftAnkleB) {
                temp_data = Roll_forward_kinematics(motor_data_[LeftAnkleA], motor_data_[LeftAnkleB]);
                data[LeftAnkleRoll].pos_ = temp_data.pos_ * LegDirectionMotor_[i];
                data[LeftAnkleRoll].vel_ = temp_data.vel_ * LegDirectionMotor_[i];
                data[LeftAnkleRoll].tau_ = temp_data.tau_ * LegDirectionMotor_[i];
            } else if (i == RightAnkleA) {
                temp_data = Pitch_forward_kinematics(motor_data_[RightAnkleA], motor_data_[RightAnkleB]);
                data[RightAnklePitch].pos_ = temp_data.pos_ * LegDirectionMotor_[i];
                data[RightAnklePitch].vel_ = temp_data.vel_ * LegDirectionMotor_[i];
                data[RightAnklePitch].tau_ = temp_data.tau_ * LegDirectionMotor_[i];
            } else if (i == RightAnkleB) {
                temp_data = Roll_forward_kinematics(motor_data_[RightAnkleA], motor_data_[RightAnkleB]);
                data[RightAnkleRoll].pos_ = temp_data.pos_ * LegDirectionMotor_[i];
                data[RightAnkleRoll].vel_ = temp_data.vel_ * LegDirectionMotor_[i];
                data[RightAnkleRoll].tau_ = temp_data.tau_ * LegDirectionMotor_[i];
            } else {
                data[i].pos_ = motor_data_[i].pos_ * LegDirectionMotor_[i];
                data[i].vel_ = motor_data_[i].vel_ * LegDirectionMotor_[i];
                data[i].tau_ = motor_data_[i].tau_ * LegDirectionMotor_[i];
            }
        }
    } else {
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            data[i].pos_ = motor_data_[i].pos_ * LegDirectionMotor_[i];
            data[i].vel_ = motor_data_[i].vel_ * LegDirectionMotor_[i];
            data[i].tau_ = motor_data_[i].tau_ * LegDirectionMotor_[i];
        }
    }
    
}
