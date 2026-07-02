//
// Created by gold on 25-3-1.
//

#include "Z1Legs.h"
#include <limits>
#include <iostream>
#include <chrono>
#include <stdexcept>
#include <cstring>
extern std::atomic<bool> stop_thread;

namespace {
constexpr double kDetEps = 1e-6;
constexpr double kSolveDenominatorEps = 1e-9;
constexpr double kForwardTol = 1e-4;
constexpr double kForwardResidualGiveUp = 0.5;
constexpr double kMaxPoseAbs = 2.0;
constexpr double kNewtonStepSoftMax = 0.2;
constexpr double kNewtonStepHardMax = 1.0;
constexpr int kLeftLegDirection = 0;


double clampUnit(const double value) {
    if (value > 1.0) {
        return 1.0;
    }
    if (value < -1.0) {
        return -1.0;
    }
    return value;
}

bool finite2(const double a, const double b) {
    return std::isfinite(a) && std::isfinite(b);
}

double finiteOrZeroLocal(const double value) {
    return std::isfinite(value) ? value : 0.0;
}

double finiteOrZero(const double value) {
    return finiteOrZeroLocal(value);
}

double clampAbs(const double value, const double max_abs) {
    if (value > max_abs) {
        return max_abs;
    }
    if (value < -max_abs) {
        return -max_abs;
    }
    return value;
}

bool poseLooksReasonable(const double roll, const double pitch) {
    return finite2(roll, pitch) && fabs(roll) <= kMaxPoseAbs && fabs(pitch) <= kMaxPoseAbs;
}

bool jacobianLooksFinite(const double J[2][2]) {
    return std::isfinite(J[0][0]) && std::isfinite(J[0][1])
        && std::isfinite(J[1][0]) && std::isfinite(J[1][1]);
}

void zeroPosVelTau(YKSMotorData& data) {
    data.pos_ = 0.0;
    data.vel_ = 0.0;
    data.tau_ = 0.0;
}

void zeroVelTau(YKSMotorData& data) {
    data.vel_ = 0.0;
    data.tau_ = 0.0;
}

struct JointCommandLimit {
    int joint_id;
    const char *name;
    double pos_min;
    double pos_max;
};

#define Z1_SAFE_JOINT_COMMAND_LIMIT_TABLE(X) \
    X(LeftHipPitch, 0, -2.268934, 2.268934) \
    X(LeftHipRoll, 1, -0.436334, 1.658034) \
    X(LeftHipYaw, 2, -2.705234, 2.705234) \
    X(LeftKnee, 3, -0.000034, 2.094434) \
    X(LeftAnklePitch, 4, -0.698134, 0.209434) \
    X(LeftAnkleRoll, 5, -0.279234, 0.279234) \
    X(RightHipPitch, 8, -2.268934, 2.268934) \
    X(RightHipRoll, 9, -1.658034, 0.436334) \
    X(RightHipYaw, 10, -2.705234, 2.705234) \
    X(RightKnee, 11, -0.000034, 2.094434) \
    X(RightAnklePitch, 12, -0.698134, 0.209434) \
    X(RightAnkleRoll, 13, -0.279234, 0.279234) \
    X(WaistRoll, 16, -0.436334, 0.436334) \
    X(WaistPitch, 17, -0.174534, 0.349034) \
    X(WaistYaw, 18, -2.705234, 2.705234) \
    X(LeftShoulderPitch, 24, -3.001934, 2.583134) \
    X(LeftShoulderRoll, 25, -0.174534, 2.007134) \
    X(LeftShoulderYaw, 26, -2.705234, 2.705234) \
    X(LeftElbow, 27, -0.418834, 2.530634) \
    X(LeftForearmRoll, 28, -2.705234, 2.705234) \
    X(LeftWristPitch, 29, -1.309034, 1.309034) \
    X(LeftWristYaw, 30, -1.483534, 1.483534) \
    X(RightShoulderPitch, 32, -3.001934, 2.583134) \
    X(RightShoulderRoll, 33, -2.007134, 0.174534) \
    X(RightShoulderYaw, 34, -2.705234, 2.705234) \
    X(RightElbow, 35, -0.418834, 2.530634) \
    X(RightForearmRoll, 36, -2.705234, 2.705234) \
    X(RightWristPitch, 37, -1.309034, 1.309034) \
    X(RightWristYaw, 38, -1.483534, 1.483534)

static const JointCommandLimit kSafeJointCommandLimits[] = {
#define X(name, joint_id, pos_min, pos_max) {joint_id, #name, pos_min, pos_max},
    Z1_SAFE_JOINT_COMMAND_LIMIT_TABLE(X)
#undef X
};

void applyJointPositionLimits(YKSMotorData *data) {
    for (const auto &limit : kSafeJointCommandLimits) {
        double &pos_des = data[limit.joint_id].pos_des_;
        if (pos_des > limit.pos_max) {
            pos_des = limit.pos_max;
        } else if (pos_des < limit.pos_min) {
            pos_des = limit.pos_min;
        }
    }
}

struct Vec3 {
    double x;
    double y;
    double z;
};

struct ParallelLinkageGeometry {
    Vec3 c1_0;
    Vec3 c2_0;
    Vec3 a1;
    Vec3 a2;
    Vec3 b1_0;
    Vec3 b2_0;
    Vec3 motor_axis;
    double rod_length1;
    double rod_length2;
    int branch1;
    int branch2;
};

struct ThetaPair {
    double theta1;
    double theta2;
};

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const double s, const Vec3& v) {
    return {s * v.x, s * v.y, s * v.z};
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Vec3 normalizeOrZero(const Vec3& v) {
    const double n = sqrt(dot(v, v));
    return (n > 1e-12) ? (1.0 / n) * v : Vec3{0.0, 0.0, 0.0};
}

Vec3 footPoint(const Vec3& c0, const double roll, const double pitch) {
    const double cos_p = cos(pitch), sin_p = sin(pitch);
    const double sin_r = sin(roll), cos_r = cos(roll);
    return {
        cos_p * c0.x + sin_r * sin_p * c0.y + cos_r * sin_p * c0.z,
        cos_r * c0.y - sin_r * c0.z,
        -sin_p * c0.x + sin_r * cos_p * c0.y + cos_r * cos_p * c0.z
    };
}

double solveMotorAngle(
    const Vec3& c,
    const Vec3& motor_center,
    const Vec3& crank_zero,
    const Vec3& motor_axis,
    const double rod_length,
    const int branch) {
    const Vec3 u = normalizeOrZero(motor_axis);
    const Vec3 d = c - motor_center;
    const double p_parallel_len = dot(u, crank_zero);
    const Vec3 p_parallel = p_parallel_len * u;
    const Vec3 p_perp = crank_zero - p_parallel;
    const Vec3 u_cross_p = cross(u, crank_zero);

    const double a = dot(d, p_perp);
    const double b = dot(d, u_cross_p);
    const double c_rhs = 0.5 * (dot(d, d) + dot(crank_zero, crank_zero) - rod_length * rod_length)
                         - dot(d, p_parallel);
    const double denominator = hypot(a, b);
    if (denominator < kSolveDenominatorEps || !std::isfinite(denominator) || !std::isfinite(c_rhs)) {
        return 0.0;
    }

    const double ratio = c_rhs / denominator;
    if (!std::isfinite(ratio)) {
        return 0.0;
    }

    const double phi = atan2(b, a);
    // Keep the legacy clamp behavior: it prevents bad display values, but does not mean the target is reachable.
    const double alpha = acos(clampUnit(ratio));
    const double theta = (branch >= 0) ? (phi + alpha) : (phi - alpha);
    return finiteOrZeroLocal(theta);
}

ParallelLinkageGeometry mirrorY(ParallelLinkageGeometry geometry) {
    geometry.c1_0.y = -geometry.c1_0.y;
    geometry.c2_0.y = -geometry.c2_0.y;
    geometry.a1.y = -geometry.a1.y;
    geometry.a2.y = -geometry.a2.y;
    geometry.b1_0.y = -geometry.b1_0.y;
    geometry.b2_0.y = -geometry.b2_0.y;
    geometry.motor_axis.y = -geometry.motor_axis.y;
    geometry.branch1 = -geometry.branch1;
    geometry.branch2 = -geometry.branch2;
    return geometry;
}

const ParallelLinkageGeometry& rightAnkleGeometry() {
    static const ParallelLinkageGeometry geometry = {
        {-45.0, 21.65, 12.5},
        {-45.0, -21.65, 12.5},
        {-44.0, 0.0, 215.0},
        {-44.0, 0.0, 155.0},
        {-44.0, 21.65, 227.5},
        {-44.0, -21.65, 167.5},
        {-1.0, 0.0, 0.0},
        215.002326,
        155.003226,
        -1,
        +1
    };
    return geometry;
}

const ParallelLinkageGeometry& leftAnkleGeometry() {
    static const ParallelLinkageGeometry geometry = mirrorY(rightAnkleGeometry());
    return geometry;
}

const ParallelLinkageGeometry& ankleGeometryForDirection(const int direction) {
    return (direction == kLeftLegDirection) ? leftAnkleGeometry() : rightAnkleGeometry();
}

// Ankle linkage: right leg uses the base geometry, left leg mirrors it in Y.
// theta1/theta2 map directly to the two ankle motor slots selected by direction.

const ParallelLinkageGeometry& waistGeometry() {
    static const ParallelLinkageGeometry geometry = {
        {-44.0, 20.0, -10.0},
        {-44.0, -20.0, -10.0},
        {-44.0, 35.0, 50.0},
        {-44.0, -35.0, 50.0},
        {-44.0, 20.86, 54.99},
        {-44.0, -20.86, 54.99},
        {-1.0, 0.0, 0.0},
        64.99569,
        64.99569,
        +1,
        -1
    };
    return geometry;
}

ThetaPair inverseForGeometry(const ParallelLinkageGeometry& geometry, const double roll, const double pitch) {
    const Vec3 c1 = footPoint(geometry.c1_0, roll, pitch);
    const Vec3 c2 = footPoint(geometry.c2_0, roll, pitch);

    const double theta1 = solveMotorAngle(
        c1, geometry.a1, geometry.b1_0 - geometry.a1,
        geometry.motor_axis, geometry.rod_length1, geometry.branch1);
    const double theta2 = solveMotorAngle(
        c2, geometry.a2, geometry.b2_0 - geometry.a2,
        geometry.motor_axis, geometry.rod_length2, geometry.branch2);
    return {theta1, theta2};
}

void computeJacobianForGeometry(
    const ParallelLinkageGeometry& geometry,
    const double roll,
    const double pitch,
    double J[2][2],
    const double delta) {
    J[0][0] = J[0][1] = J[1][0] = J[1][1] = 0.0;
    if (!finite2(roll, pitch) || !std::isfinite(delta) || fabs(delta) < 1e-12) {
        return;
    }

    auto theta_p = inverseForGeometry(geometry, roll + delta, pitch);
    auto theta_m = inverseForGeometry(geometry, roll - delta, pitch);
    J[0][0] = finiteOrZeroLocal((theta_p.theta1 - theta_m.theta1) / (2 * delta));
    J[1][0] = finiteOrZeroLocal((theta_p.theta2 - theta_m.theta2) / (2 * delta));

    theta_p = inverseForGeometry(geometry, roll, pitch + delta);
    theta_m = inverseForGeometry(geometry, roll, pitch - delta);
    J[0][1] = finiteOrZeroLocal((theta_p.theta1 - theta_m.theta1) / (2 * delta));
    J[1][1] = finiteOrZeroLocal((theta_p.theta2 - theta_m.theta2) / (2 * delta));
}

void inverseVelocityForGeometry(
    const ParallelLinkageGeometry& geometry,
    const double roll,
    const double pitch,
    const double end_vel[2],
    double joint_vel[2]) {
    joint_vel[0] = joint_vel[1] = 0.0;
    if (!finite2(roll, pitch) || !finite2(end_vel[0], end_vel[1])) {
        return;
    }

    double J[2][2];
    computeJacobianForGeometry(geometry, roll, pitch, J, 1e-6);
    if (!jacobianLooksFinite(J)) {
        return;
    }

    joint_vel[0] = finiteOrZeroLocal(J[0][0] * end_vel[0] + J[0][1] * end_vel[1]);
    joint_vel[1] = finiteOrZeroLocal(J[1][0] * end_vel[0] + J[1][1] * end_vel[1]);
}

void forwardVelocityForGeometry(
    const ParallelLinkageGeometry& geometry,
    const double roll,
    const double pitch,
    const double joint_vel[2],
    double end_vel[2]) {
    end_vel[0] = end_vel[1] = 0.0;
    if (!finite2(roll, pitch) || !finite2(joint_vel[0], joint_vel[1])) {
        return;
    }

    double J[2][2];
    computeJacobianForGeometry(geometry, roll, pitch, J, 1e-6);
    if (!jacobianLooksFinite(J)) {
        return;
    }

    const double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (!std::isfinite(det) || fabs(det) < kDetEps) {
        return;
    }

    const double invJ[2][2] = {
        { J[1][1] / det, -J[0][1] / det },
        { -J[1][0] / det, J[0][0] / det }
    };
    end_vel[0] = finiteOrZeroLocal(invJ[0][0] * joint_vel[0] + invJ[0][1] * joint_vel[1]);
    end_vel[1] = finiteOrZeroLocal(invJ[1][0] * joint_vel[0] + invJ[1][1] * joint_vel[1]);
}

void torqueToMotorForGeometry(
    const ParallelLinkageGeometry& geometry,
    const double roll,
    const double pitch,
    const double end_tau[2],
    double joint_tau[2]) {
    joint_tau[0] = joint_tau[1] = 0.0;
    if (!finite2(roll, pitch) || !finite2(end_tau[0], end_tau[1])) {
        return;
    }

    double J[2][2];
    computeJacobianForGeometry(geometry, roll, pitch, J, 1e-6);
    if (!jacobianLooksFinite(J)) {
        return;
    }

    const double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (!std::isfinite(det) || fabs(det) < kDetEps) {
        return;
    }

    joint_tau[0] = finiteOrZeroLocal((J[1][1] * end_tau[0] - J[1][0] * end_tau[1]) / det);
    joint_tau[1] = finiteOrZeroLocal((-J[0][1] * end_tau[0] + J[0][0] * end_tau[1]) / det);
}

void motorTorqueToEndForGeometry(
    const ParallelLinkageGeometry& geometry,
    const double roll,
    const double pitch,
    const double joint_tau[2],
    double end_tau[2]) {
    end_tau[0] = end_tau[1] = 0.0;
    if (!finite2(roll, pitch) || !finite2(joint_tau[0], joint_tau[1])) {
        return;
    }

    double J[2][2];
    computeJacobianForGeometry(geometry, roll, pitch, J, 1e-6);
    if (!jacobianLooksFinite(J)) {
        return;
    }

    end_tau[0] = finiteOrZeroLocal(J[0][0] * joint_tau[0] + J[1][0] * joint_tau[1]);
    end_tau[1] = finiteOrZeroLocal(J[0][1] * joint_tau[0] + J[1][1] * joint_tau[1]);
}

void forwardForGeometry(
    const ParallelLinkageGeometry& geometry,
    const double theta1,
    const double theta2,
    double& roll,
    double& pitch) {
    if (!std::isfinite(theta1) || !std::isfinite(theta2)) {
        roll = 0.0;
        pitch = 0.0;
        return;
    }

    double current_roll = 0.0;
    double current_pitch = 0.0;
    const int max_iter = 100;
    double residual_norm = std::numeric_limits<double>::infinity();
    bool converged = false;

    for (int i = 0; i < max_iter; ++i) {
        auto current_theta = inverseForGeometry(geometry, current_roll, current_pitch);
        const double res1 = current_theta.theta1 - theta1;
        const double res2 = current_theta.theta2 - theta2;
        if (!std::isfinite(res1) || !std::isfinite(res2)) {
            break;
        }
        residual_norm = hypot(res1, res2);
        if (!std::isfinite(residual_norm)) {
            break;
        }
        if (residual_norm < kForwardTol) {
            converged = true;
            break;
        }

        double J[2][2];
        computeJacobianForGeometry(geometry, current_roll, current_pitch, J, 1e-6);
        if (!jacobianLooksFinite(J)) {
            break;
        }
        const double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
        if (!std::isfinite(det) || fabs(det) < kDetEps) {
            break;
        }

        double delta_roll = (-res1 * J[1][1] + res2 * J[0][1]) / det;
        double delta_pitch = (res1 * J[1][0] - res2 * J[0][0]) / det;
        if (!std::isfinite(delta_roll) || !std::isfinite(delta_pitch)) {
            break;
        }

        if (fabs(delta_roll) > kNewtonStepHardMax || fabs(delta_pitch) > kNewtonStepHardMax) {
            break;
        }
        delta_roll = clampAbs(delta_roll, kNewtonStepSoftMax);
        delta_pitch = clampAbs(delta_pitch, kNewtonStepSoftMax);

        current_roll += delta_roll;
        current_pitch += delta_pitch;
        if (!finite2(current_roll, current_pitch)) {
            current_roll = 0.0;
            current_pitch = 0.0;
            break;
        }
        if (!poseLooksReasonable(current_roll, current_pitch)) {
            current_roll = 0.0;
            current_pitch = 0.0;
            break;
        }
    }

    if (poseLooksReasonable(current_roll, current_pitch)) {
        const auto final_theta = inverseForGeometry(geometry, current_roll, current_pitch);
        const double final_res1 = final_theta.theta1 - theta1;
        const double final_res2 = final_theta.theta2 - theta2;
        const double final_residual = hypot(final_res1, final_res2);
        if (std::isfinite(final_residual)) {
            residual_norm = final_residual;
        }
    }

    if (!poseLooksReasonable(current_roll, current_pitch)
        || (!converged && (!std::isfinite(residual_norm) || residual_norm > kForwardResidualGiveUp))) {
        current_roll = 0.0;
        current_pitch = 0.0;
    }

    roll = finiteOrZeroLocal(current_roll);
    pitch = finiteOrZeroLocal(current_pitch);
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

void Z1Legs::PrintMotorState(const int /*size*/) const {

    // 打印 flag 状态
    attron(COLOR_PAIR(3));
    mvprintw(2, 2, "IMU: %s", is_imu_run_ ? "OK" : "ERR");
    mvprintw(2, 12, "Hcmd: %s", is_hcmd_run_ ? "OK" : "ERR");
    mvprintw(2, 22, "Logic: %s", is_logic_run_ ? "OK" : "ERR");
    mvprintw(2, 36, "Bms: %s", is_bms_run_ ? "OK" : "ERR");
    if (battery_enable_) {
        const BmsState state = battery_handler_->getState();
        mvprintw(2, 48, "Soc: %d%% Temp: %.2f", state.soc, state.temperature);
    }
    attroff(COLOR_PAIR(3));

    mvprintw(4, 0, "ID  | Joint                | Pos      | Vel      | Tau      | DesPos   | DesVel   | KP    | KD    | FF");

    mvprintw(5, 0,
             "------------------------------------------------------------------------------------------------------------------");

    // 打印电机状态
    for (int row = 0; row < ACTIVE_MOTOR_NUMBER; ++row) {
        const int i = kAllActiveIds[row];
        move(row + 6, 0);
        clrtoeol();

        attron(COLOR_PAIR(1));
        mvprintw(row + 6, 0, "%2d", i);
        attroff(COLOR_PAIR(1));

        mvprintw(row + 6, 6, "%-20s", kJointNames[i]);

        attron(COLOR_PAIR(2));
        mvprintw(row + 6, 28, "%9.3f", finiteOrZero(motor_print_[i].pos_));
        attroff(COLOR_PAIR(2));

        attron(COLOR_PAIR(3));
        mvprintw(row + 6, 39, "%9.3f", finiteOrZero(motor_print_[i].vel_));
        attroff(COLOR_PAIR(3));

        attron(COLOR_PAIR(4));
        mvprintw(row + 6, 50, "%9.3f", finiteOrZero(motor_print_[i].tau_));
        attroff(COLOR_PAIR(4));

        attron(COLOR_PAIR(5));
        mvprintw(row + 6, 61, "%9.3f", finiteOrZero(motor_print_[i].pos_des_));
        attroff(COLOR_PAIR(5));

        attron(COLOR_PAIR(6));
        mvprintw(row + 6, 72, "%9.3f", finiteOrZero(motor_print_[i].vel_des_));
        attroff(COLOR_PAIR(6));

        mvprintw(row + 6, 83, "%7.3f", finiteOrZero(motor_print_[i].kp_));
        mvprintw(row + 6, 91, "%7.3f", finiteOrZero(motor_print_[i].kd_));
        mvprintw(row + 6, 99, "%7.3f", finiteOrZero(motor_print_[i].ff_));
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

Z1Legs::InverseKinematicsResult Z1Legs::waist_inverse_kinematics(double roll, double pitch) const {
    const auto theta = inverseForGeometry(waistGeometry(), roll, pitch);
    return {theta.theta1, theta.theta2};
}

void Z1Legs::waist_compute_jacobian(double roll, double pitch, double J[2][2], double delta) const {
    computeJacobianForGeometry(waistGeometry(), roll, pitch, J, delta);
}

void Z1Legs::waist_inverse_velocity(double roll, double pitch, const double end_vel[2], double joint_vel[2]) const {
    inverseVelocityForGeometry(waistGeometry(), roll, pitch, end_vel, joint_vel);
}

void Z1Legs::waist_forward_kinematics(double theta1, double theta2, double& roll, double& pitch) const {
    forwardForGeometry(waistGeometry(), theta1, theta2, roll, pitch);
}

YKSMotorData
Z1Legs::AnkleA_inverse_kinematics(const YKSMotorData &pitch_joint_cmd, const YKSMotorData &roll_joint_cmd,
                                  int direction) {
    YKSMotorData cmd;

    const auto& geometry = ankleGeometryForDirection(direction);

    const double desired_roll = roll_joint_cmd.pos_des_;
    const double desired_pitch = pitch_joint_cmd.pos_des_;
    const auto theta = inverseForGeometry(geometry, desired_roll, desired_pitch);
    cmd.pos_des_ = theta.theta1;

    const double end_vel[2] = {roll_joint_cmd.vel_des_, pitch_joint_cmd.vel_des_};
    double joint_vel[2];
    inverseVelocityForGeometry(geometry, desired_roll, desired_pitch, end_vel, joint_vel);
    cmd.vel_des_ = joint_vel[0];

    const double end_tau[2] = {roll_joint_cmd.ff_, pitch_joint_cmd.ff_};
    double joint_tau[2];
    torqueToMotorForGeometry(geometry, desired_roll, desired_pitch, end_tau, joint_tau);
    cmd.ff_ = joint_tau[0];
    return cmd;
}

YKSMotorData
Z1Legs::AnkleB_inverse_kinematics(const YKSMotorData &pitch_joint_cmd, const YKSMotorData &roll_joint_cmd,
                                  int direction) {
    YKSMotorData cmd;

    const auto& geometry = ankleGeometryForDirection(direction);

    const double desired_roll = roll_joint_cmd.pos_des_;
    const double desired_pitch = pitch_joint_cmd.pos_des_;
    const auto theta = inverseForGeometry(geometry, desired_roll, desired_pitch);
    cmd.pos_des_ = theta.theta2;

    const double end_vel[2] = {roll_joint_cmd.vel_des_, pitch_joint_cmd.vel_des_};
    double joint_vel[2];
    inverseVelocityForGeometry(geometry, desired_roll, desired_pitch, end_vel, joint_vel);
    cmd.vel_des_ = joint_vel[1];

    const double end_tau[2] = {roll_joint_cmd.ff_, pitch_joint_cmd.ff_};
    double joint_tau[2];
    torqueToMotorForGeometry(geometry, desired_roll, desired_pitch, end_tau, joint_tau);
    cmd.ff_ = joint_tau[1];
    return cmd;
}

YKSMotorData Z1Legs::WaistA_inverse_kinematics(
    const YKSMotorData &pitch_joint_cmd,
    const YKSMotorData &roll_joint_cmd) {
    YKSMotorData cmd;

    const double desired_roll = roll_joint_cmd.pos_des_;
    const double desired_pitch = -pitch_joint_cmd.pos_des_;
    auto theta = waist_inverse_kinematics(desired_roll, desired_pitch);
    cmd.pos_des_ = theta.theta1;

    const double end_vel[2] = {roll_joint_cmd.vel_des_, -pitch_joint_cmd.vel_des_};
    double joint_vel[2];
    waist_inverse_velocity(desired_roll, desired_pitch, end_vel, joint_vel);
    cmd.vel_des_ = joint_vel[0];

    const double solver_roll_ff = roll_joint_cmd.ff_;
    const double solver_pitch_ff = -pitch_joint_cmd.ff_;
    const double end_tau[2] = {solver_roll_ff, solver_pitch_ff};
    double joint_tau[2];
    torqueToMotorForGeometry(waistGeometry(), desired_roll, desired_pitch, end_tau, joint_tau);
    cmd.ff_ = joint_tau[0];
    return cmd;
}

YKSMotorData Z1Legs::WaistB_inverse_kinematics(
    const YKSMotorData &pitch_joint_cmd,
    const YKSMotorData &roll_joint_cmd) {
    YKSMotorData cmd;

    const double desired_roll = roll_joint_cmd.pos_des_;
    const double desired_pitch = -pitch_joint_cmd.pos_des_;
    auto theta = waist_inverse_kinematics(desired_roll, desired_pitch);
    cmd.pos_des_ = theta.theta2;

    const double end_vel[2] = {roll_joint_cmd.vel_des_, -pitch_joint_cmd.vel_des_};
    double joint_vel[2];
    waist_inverse_velocity(desired_roll, desired_pitch, end_vel, joint_vel);
    cmd.vel_des_ = joint_vel[1];

    const double solver_roll_ff = roll_joint_cmd.ff_;
    const double solver_pitch_ff = -pitch_joint_cmd.ff_;
    const double end_tau[2] = {solver_roll_ff, solver_pitch_ff};
    double joint_tau[2];
    torqueToMotorForGeometry(waistGeometry(), desired_roll, desired_pitch, end_tau, joint_tau);
    cmd.ff_ = joint_tau[1];
    return cmd;
}

YKSMotorData Z1Legs::WaistPitch_forward_kinematics(
    const YKSMotorData &waist_a_motor,
    const YKSMotorData &waist_b_motor) {
    YKSMotorData data;

    // Waist pitch uses the opposite sign of the linkage solver pitch.
    const double theta1 = waist_a_motor.pos_;
    const double theta2 = waist_b_motor.pos_;
    if (!finite2(theta1, theta2)) {
        zeroPosVelTau(data);
        return data;
    }

    double roll, pitch;
    waist_forward_kinematics(theta1, theta2, roll, pitch);
    if (!poseLooksReasonable(roll, pitch)) {
        zeroPosVelTau(data);
        return data;
    }
    data.pos_ = finiteOrZeroLocal(-pitch);

    double J[2][2];
    waist_compute_jacobian(roll, pitch, J);
    if (!jacobianLooksFinite(J)) {
        zeroVelTau(data);
        return data;
    }

    const double joint_vel[2] = {waist_a_motor.vel_, waist_b_motor.vel_};
    double solver_vel[2];
    forwardVelocityForGeometry(waistGeometry(), roll, pitch, joint_vel, solver_vel);
    data.vel_ = -solver_vel[1];

    const double joint_tau[2] = {waist_a_motor.tau_, waist_b_motor.tau_};
    double solver_tau[2];
    motorTorqueToEndForGeometry(waistGeometry(), roll, pitch, joint_tau, solver_tau);
    data.tau_ = -solver_tau[1];

    return data;
}

YKSMotorData Z1Legs::WaistRoll_forward_kinematics(
    const YKSMotorData &waist_a_motor,
    const YKSMotorData &waist_b_motor) {
    YKSMotorData data;

    const double theta1 = waist_a_motor.pos_;
    const double theta2 = waist_b_motor.pos_;
    if (!finite2(theta1, theta2)) {
        zeroPosVelTau(data);
        return data;
    }

    double roll, pitch;
    waist_forward_kinematics(theta1, theta2, roll, pitch);
    if (!poseLooksReasonable(roll, pitch)) {
        zeroPosVelTau(data);
        return data;
    }
    data.pos_ = finiteOrZeroLocal(roll);

    double J[2][2];
    waist_compute_jacobian(roll, pitch, J);
    if (!jacobianLooksFinite(J)) {
        zeroVelTau(data);
        return data;
    }

    const double joint_vel[2] = {waist_a_motor.vel_, waist_b_motor.vel_};
    double solver_vel[2];
    forwardVelocityForGeometry(waistGeometry(), roll, pitch, joint_vel, solver_vel);
    data.vel_ = solver_vel[0];

    const double joint_tau[2] = {waist_a_motor.tau_, waist_b_motor.tau_};
    double solver_tau[2];
    motorTorqueToEndForGeometry(waistGeometry(), roll, pitch, joint_tau, solver_tau);
    data.tau_ = solver_tau[0];

    return data;
}

YKSMotorData Z1Legs::Pitch_forward_kinematics(const YKSMotorData &b1_motor,
                                              const YKSMotorData &b2_motor, int direction) {
    YKSMotorData data;

    const auto& geometry = ankleGeometryForDirection(direction);

    const double theta1 = b1_motor.pos_;
    const double theta2 = b2_motor.pos_;
    if (!finite2(theta1, theta2)) {
        zeroPosVelTau(data);
        return data;
    }

    double roll, pitch;
    forwardForGeometry(geometry, theta1, theta2, roll, pitch);
    if (!poseLooksReasonable(roll, pitch)) {
        zeroPosVelTau(data);
        return data;
    }
    data.pos_ = finiteOrZeroLocal(pitch);

    double J[2][2];
    computeJacobianForGeometry(geometry, roll, pitch, J, 1e-6);
    if (!jacobianLooksFinite(J)) {
        zeroVelTau(data);
        return data;
    }
    const double joint_vel[2] = {b1_motor.vel_, b2_motor.vel_};
    double end_vel[2];
    forwardVelocityForGeometry(geometry, roll, pitch, joint_vel, end_vel);
    data.vel_ = end_vel[1];

    const double joint_tau[2] = {b1_motor.tau_, b2_motor.tau_};
    double end_tau[2];
    motorTorqueToEndForGeometry(geometry, roll, pitch, joint_tau, end_tau);
    data.tau_ = end_tau[1];

    return data;
}

YKSMotorData Z1Legs::Roll_forward_kinematics(const YKSMotorData &b1_motor,
                                             const YKSMotorData &b2_motor, int direction) {
    YKSMotorData data;

    const auto& geometry = ankleGeometryForDirection(direction);

    const double theta1 = b1_motor.pos_;
    const double theta2 = b2_motor.pos_;
    if (!finite2(theta1, theta2)) {
        zeroPosVelTau(data);
        return data;
    }

    double roll, pitch;
    forwardForGeometry(geometry, theta1, theta2, roll, pitch);
    if (!poseLooksReasonable(roll, pitch)) {
        zeroPosVelTau(data);
        return data;
    }
    data.pos_ = finiteOrZeroLocal(roll);

    double J[2][2];
    computeJacobianForGeometry(geometry, roll, pitch, J, 1e-6);
    if (!jacobianLooksFinite(J)) {
        zeroVelTau(data);
        return data;
    }
    const double joint_vel[2] = {b1_motor.vel_, b2_motor.vel_};
    double end_vel[2];
    forwardVelocityForGeometry(geometry, roll, pitch, joint_vel, end_vel);
    data.vel_ = end_vel[0];

    const double joint_tau[2] = {b1_motor.tau_, b2_motor.tau_};
    double end_tau[2];
    motorTorqueToEndForGeometry(geometry, roll, pitch, joint_tau, end_tau);
    data.tau_ = end_tau[0];

    return data;
}

//以上都是私有函数，不能从外部调用
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//以下都是公有函数，可以从外部调用

Z1Legs::~Z1Legs() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::cout << "Z1Legs Destructor "  << std::endl;
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
    YKSMotorData limited_data[Z1_NUM_MOTOR];
    std::memcpy(limited_data, data, Z1_NUM_MOTOR * sizeof(YKSMotorData));
    applyJointPositionLimits(limited_data);

    std::lock_guard lock(mutex_);
    if (mode_pr_ == Mode::PR) {
        YKSMotorData tempData;
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            motor_data_[i].mode = limited_data[i].mode;
            motor_print_[i].mode = limited_data[i].mode;
            if (i == LeftAnklePitch) {
                tempData = AnkleA_inverse_kinematics(limited_data[LeftAnklePitch], limited_data[LeftAnkleRoll], LeftLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_;
                motor_data_[i].vel_des_ = tempData.vel_des_;
                motor_data_[i].ff_ = tempData.ff_;
                motor_print_[i].pos_des_ = limited_data[i].pos_des_;
                motor_print_[i].vel_des_ = limited_data[i].vel_des_;
                motor_print_[i].ff_ = limited_data[i].ff_;
            } else if (i == LeftAnkleRoll) {
                tempData = AnkleB_inverse_kinematics(limited_data[LeftAnklePitch], limited_data[LeftAnkleRoll], LeftLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_;
                motor_data_[i].vel_des_ = tempData.vel_des_;
                motor_data_[i].ff_ = tempData.ff_;
                motor_print_[i].pos_des_ = limited_data[i].pos_des_;
                motor_print_[i].vel_des_ = limited_data[i].vel_des_;
                motor_print_[i].ff_ = limited_data[i].ff_;
            } else if (i == RightAnklePitch) {
                tempData = AnkleA_inverse_kinematics(limited_data[RightAnklePitch], limited_data[RightAnkleRoll], RightLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_;
                motor_data_[i].vel_des_ = tempData.vel_des_;
                motor_data_[i].ff_ = tempData.ff_;
                motor_print_[i].pos_des_ = limited_data[i].pos_des_;
                motor_print_[i].vel_des_ = limited_data[i].vel_des_;
                motor_print_[i].ff_ = limited_data[i].ff_;
            } else if (i == RightAnkleRoll) {
                tempData = AnkleB_inverse_kinematics(limited_data[RightAnklePitch], limited_data[RightAnkleRoll], RightLeg);
                motor_data_[i].pos_des_ = tempData.pos_des_;
                motor_data_[i].vel_des_ = tempData.vel_des_;
                motor_data_[i].ff_ = tempData.ff_;
                motor_print_[i].pos_des_ = limited_data[i].pos_des_;
                motor_print_[i].vel_des_ = limited_data[i].vel_des_;
                motor_print_[i].ff_ = limited_data[i].ff_;
            } else if (i == WaistA) {
                tempData = WaistA_inverse_kinematics(limited_data[WaistPitch], limited_data[WaistRoll]);
                motor_data_[i].pos_des_ = tempData.pos_des_;
                motor_data_[i].vel_des_ = tempData.vel_des_;
                motor_data_[i].ff_ = tempData.ff_;
                motor_print_[i].pos_des_ = limited_data[i].pos_des_;
                motor_print_[i].vel_des_ = limited_data[i].vel_des_;
                motor_print_[i].ff_ = limited_data[i].ff_;
            } else if (i == WaistB) {
                tempData = WaistB_inverse_kinematics(limited_data[WaistPitch], limited_data[WaistRoll]);
                motor_data_[i].pos_des_ = tempData.pos_des_;
                motor_data_[i].vel_des_ = tempData.vel_des_;
                motor_data_[i].ff_ = tempData.ff_;
                motor_print_[i].pos_des_ = limited_data[i].pos_des_;
                motor_print_[i].vel_des_ = limited_data[i].vel_des_;
                motor_print_[i].ff_ = limited_data[i].ff_;
            } else {
                motor_data_[i].pos_des_ = limited_data[i].pos_des_ * LegDirectionMotor_[i];
                motor_data_[i].vel_des_ = limited_data[i].vel_des_ * LegDirectionMotor_[i];
                motor_data_[i].ff_ = limited_data[i].ff_ * LegDirectionMotor_[i];
                motor_print_[i].pos_des_ = limited_data[i].pos_des_;
                motor_print_[i].vel_des_ = limited_data[i].vel_des_;
                motor_print_[i].ff_ = limited_data[i].ff_;
            }
        }
    } else {
        for (int i = 0; i < Z1_NUM_MOTOR; ++i) {
            motor_data_[i].mode = limited_data[i].mode;
            motor_data_[i].pos_des_ = limited_data[i].pos_des_ * LegDirectionMotor_[i];
            motor_data_[i].vel_des_ = limited_data[i].vel_des_ * LegDirectionMotor_[i];
            motor_data_[i].ff_ = limited_data[i].ff_ * LegDirectionMotor_[i];
            motor_print_[i].mode = limited_data[i].mode;
            motor_print_[i].pos_des_ = limited_data[i].pos_des_;
            motor_print_[i].vel_des_ = limited_data[i].vel_des_;
            motor_print_[i].ff_ = limited_data[i].ff_;
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
            if (i == LeftAnklePitch) {
                temp_data = Pitch_forward_kinematics(motor_data_[LeftAnklePitch], motor_data_[LeftAnkleRoll], LeftLeg);
                data[LeftAnklePitch].pos_ = temp_data.pos_;
                data[LeftAnklePitch].vel_ = temp_data.vel_;
                data[LeftAnklePitch].tau_ = temp_data.tau_;
                motor_print_[LeftAnklePitch].pos_ = temp_data.pos_;
                motor_print_[LeftAnklePitch].vel_ = temp_data.vel_;
                motor_print_[LeftAnklePitch].tau_ = temp_data.tau_;
            } else if (i == LeftAnkleRoll) {
                temp_data = Roll_forward_kinematics(motor_data_[LeftAnklePitch], motor_data_[LeftAnkleRoll], LeftLeg);
                data[LeftAnkleRoll].pos_ = temp_data.pos_;
                data[LeftAnkleRoll].vel_ = temp_data.vel_;
                data[LeftAnkleRoll].tau_ = temp_data.tau_;
                motor_print_[LeftAnkleRoll].pos_ = temp_data.pos_;
                motor_print_[LeftAnkleRoll].vel_ = temp_data.vel_;
                motor_print_[LeftAnkleRoll].tau_ = temp_data.tau_;
            } else if (i == RightAnklePitch) {
                temp_data = Pitch_forward_kinematics(motor_data_[RightAnklePitch], motor_data_[RightAnkleRoll], RightLeg);
                data[RightAnklePitch].pos_ = temp_data.pos_;
                data[RightAnklePitch].vel_ = temp_data.vel_;
                data[RightAnklePitch].tau_ = temp_data.tau_;
                motor_print_[RightAnklePitch].pos_ = temp_data.pos_;
                motor_print_[RightAnklePitch].vel_ = temp_data.vel_;
                motor_print_[RightAnklePitch].tau_ = temp_data.tau_;
            } else if (i == RightAnkleRoll) {
                temp_data = Roll_forward_kinematics(motor_data_[RightAnklePitch], motor_data_[RightAnkleRoll], RightLeg);
                data[RightAnkleRoll].pos_ = temp_data.pos_;
                data[RightAnkleRoll].vel_ = temp_data.vel_;
                data[RightAnkleRoll].tau_ = temp_data.tau_;
                motor_print_[RightAnkleRoll].pos_ = temp_data.pos_;
                motor_print_[RightAnkleRoll].vel_ = temp_data.vel_;
                motor_print_[RightAnkleRoll].tau_ = temp_data.tau_;
            } else if (i == WaistA) {
                temp_data = WaistRoll_forward_kinematics(motor_data_[WaistA], motor_data_[WaistB]);
                data[WaistRoll].pos_ = temp_data.pos_;
                data[WaistRoll].vel_ = temp_data.vel_;
                data[WaistRoll].tau_ = temp_data.tau_;
                motor_print_[WaistRoll].pos_ = temp_data.pos_;
                motor_print_[WaistRoll].vel_ = temp_data.vel_;
                motor_print_[WaistRoll].tau_ = temp_data.tau_;
            } else if (i == WaistB) {
                temp_data = WaistPitch_forward_kinematics(motor_data_[WaistA], motor_data_[WaistB]);
                data[WaistPitch].pos_ = temp_data.pos_;
                data[WaistPitch].vel_ = temp_data.vel_;
                data[WaistPitch].tau_ = temp_data.tau_;
                motor_print_[WaistPitch].pos_ = temp_data.pos_;
                motor_print_[WaistPitch].vel_ = temp_data.vel_;
                motor_print_[WaistPitch].tau_ = temp_data.tau_;
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
