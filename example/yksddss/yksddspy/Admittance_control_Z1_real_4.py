#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
本脚本为Z1机器人实现一个混合控制方案。

控制策略如下：
1.  对于左右臂共10个关节：
    - 使用Pinocchio计算重力补偿。
    - 结合光滑摩擦模型（库仑+粘性）进行力矩补偿。
    - 实时计算非线性阻尼系数并赋值给kd，实现非线性阻抗控制。
2.  对于其余20个关节：使用PD控制器使其保持在零位。
"""

import time
import numpy as np
import pinocchio as pin
from z1_test import Z1RemoteClient, WHOLEBODYCMDTOPIC, WHOLEBODYSTATETOPIC

# --- 1. 用户配置 ---

# 【！】为手臂关节配置非线性阻抗参数
NONLINEAR_MU = 4.2
NONLINEAR_N = 1.7

# 【！】为手臂关节配置基础PD增益
ARM_KP = 0.0

# 【！】新增：为手臂关节配置摩擦补偿参数
# 这是解决静止时下落问题的关键
COULOMB_FRICTION_MAGNITUDE = 0.3  # (μ_c) 库仑摩擦大小，用于抵消残余重力。从0.1开始慢慢调试。
VISCOUS_FRICTION_COEFFICIENT = 0.4 # (μ_v) 粘性摩擦系数，增加高速阻尼。
FRICTION_VELOCITY_THRESHOLD = 0.04 # (ε_v) 速度阈值，用于平滑库仑摩擦，防止振动。

# 【！】为其余保持零位的关节配置PD增益
ZERO_POS_KP = 50.0
ZERO_POS_KD = 1.0

# 关节索引定义 (无需修改)
GRAV_COMP_INDICES = [24, 12, 13, 14, 15,  25, 18, 19, 20, 21]
MODELED_DDS_INDICES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 26, 27, 24, 12, 13, 14, 15, 25, 18, 19, 20, 21]
ALL_DDS_INDICES = set(range(30))
ZERO_POS_INDICES = sorted(list(ALL_DDS_INDICES - set(GRAV_COMP_INDICES)))

Z1_MODEL_PATH = "/home/amov/humanoid_proj/z1_rl/z1.5/z1.5_description_0529_0.xml"

def main():
    # --- 2. 初始化 ---
    NUM_MOTORS = 30
    print("正在初始化Z1远程客户端...")
    z1_robot = Z1RemoteClient(WHOLEBODYCMDTOPIC, WHOLEBODYSTATETOPIC, 'Z1_5_WB')
    time.sleep(1.0)

    print(f"正在加载24自由度模型: {Z1_MODEL_PATH}")
    model = pin.buildModelFromMJCF(Z1_MODEL_PATH)
    data = model.createData()

    # ... (省略部分不变的初始化代码) ...
    if model.nv != len(MODELED_DDS_INDICES): return
        
    print(f"模型加载成功，包含 {model.nv} 个自由度。")
    print(f"非线性阻抗及摩擦补偿关节 (手臂): {GRAV_COMP_INDICES}")
    
    dds_to_pin_idx_map = {dds_idx: pin_idx for pin_idx, dds_idx in enumerate(MODELED_DDS_INDICES)}
    q_all = np.zeros(NUM_MOTORS)
    v_all = np.zeros(NUM_MOTORS)

    print("\n启动混合控制循环（增加摩擦补偿）...")
    print("按 Ctrl+C 停止。")

    # --- 3. 主控制循环 ---
    try:
        while True:
            loop_start_time = time.time()
            motor_states = z1_robot.getStates()
            if len(motor_states.states) != NUM_MOTORS:
                time.sleep(0.001)
                continue

            for i in range(NUM_MOTORS):
                q_all[i] = motor_states.states[i].pos
                v_all[i] = motor_states.states[i].vel

            q_modeled = q_all[MODELED_DDS_INDICES]
            tau_gravity_24 = pin.computeGeneralizedGravity(model, data, q_modeled)

            for i in range(NUM_MOTORS):
                cmd = z1_robot.motorCmds.cmds[i]
                cmd.mode = 0
                
                if i in GRAV_COMP_INDICES:
                    v_current = v_all[i]
                    # --- A. 计算非线性阻尼 ---
                    v_error = -v_current
                    epsilon = 1e-5
                    kd_nonlinear = NONLINEAR_MU * (np.abs(v_error) + epsilon)**(NONLINEAR_N - 1)
                    
                    # --- B. 计算摩擦补偿力矩 ---
                    # 光滑的库仑摩擦项
                    tau_coulomb = COULOMB_FRICTION_MAGNITUDE * np.tanh(v_current / FRICTION_VELOCITY_THRESHOLD)
                    # 粘性摩擦项 (注意方向，摩擦力与速度方向相反)
                    tau_viscous = VISCOUS_FRICTION_COEFFICIENT * v_current
                    # 总摩擦力矩 (方向与速度相反，因此加在tau上时应该为负)
                    tau_friction = tau_coulomb + tau_viscous

                    # --- C. 配置最终指令 ---
                    cmd.kp = ARM_KP
                    cmd.kd = kd_nonlinear # 动态计算的非线性阻尼
                    
                    # cmd.pos = q_all[i]
                    cmd.pos = 0
                    cmd.vel = 0.0
                    
                    # 最终前馈力矩 = 重力补偿力矩 - 总摩擦力矩
                    pin_idx = dds_to_pin_idx_map[i]
                    cmd.tau = tau_gravity_24[pin_idx] - tau_friction

                else:
                    # 其他关节逻辑不变
                    cmd.kp = ZERO_POS_KP
                    cmd.kd = ZERO_POS_KD
                    cmd.pos = 0.0
                    cmd.vel = 0.0
                    cmd.tau = 0.0

            z1_robot.setCommand()

            elapsed = time.time() - loop_start_time
            sleep_time = 0.002 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n正在停止混合控制循环。")
    finally:
        print("正在关闭Z1远程客户端。")
        z1_robot.stop()

if __name__ == '__main__':
    main()