#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
本脚本为Z1机器人实现一个混合控制方案。

控制策略如下：
1.  对于左右臂共10个关节：
    - 使用Pinocchio计算重力补偿作为前馈力矩。
    - 实时计算非线性阻尼系数并赋值给kd，实现非线性阻抗控制。
2.  对于其余20个关节：使用PD控制器，在力位混合控制模式下使其保持在零位。
"""

import time
import numpy as np
import pinocchio as pin
from z1_test import Z1RemoteClient, WHOLEBODYCMDTOPIC, WHOLEBODYSTATETOPIC

# --- 1. 用户配置 ---

# 【！】为手臂关节配置非线性阻抗参数 (源自 Admittance_control_2.py)
NONLINEAR_MU = 4.2  # 非线性阻尼系数，控制整体阻尼大小
NONLINEAR_N = 1.7   # 非线性指数 (1 < n < 2)，控制阻尼随速度变化的剧烈程度

# 【！】为手臂关节配置基础PD增益 (用于力位混合模式)
ARM_KP = 0.0       # 手臂关节的比例增益，提供基础刚度

# 【！】为其余保持零位的关节配置PD增益
ZERO_POS_KP = 50.0  # 零位保持关节的比例增益
ZERO_POS_KD = 1.0   # 零位保持关节的微分增益

# 【！】关节索引定义 (无需修改)
# 定义需要进行重力补偿和非线性阻抗控制的10个手臂关节
GRAV_COMP_INDICES = [24, 12, 13, 14, 15, 25, 18, 19, 20, 21]

# 机器人模型中定义的24个关节的DDS索引（用于Pinocchio计算）
MODELED_DDS_INDICES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 26, 27, 24, 12, 13, 14, 15, 25, 18, 19, 20, 21]

# 自动计算需要保持零位的关节索引
ALL_DDS_INDICES = set(range(30))
ZERO_POS_INDICES = sorted(list(ALL_DDS_INDICES - set(GRAV_COMP_INDICES)))

# 机器人模型路径
Z1_MODEL_PATH = "/home/amov/humanoid_proj/z1_rl/z1.5/z1.5_description_0529_0.xml"

def main():
    # --- 2. 初始化 ---
    n = 1

    NUM_MOTORS = 30
    
    print("正在初始化Z1远程客户端...")
    z1_robot = Z1RemoteClient(WHOLEBODYCMDTOPIC, WHOLEBODYSTATETOPIC, 'Z1_5_WB')
    time.sleep(1.0)

    print(f"正在加载24自由度模型: {Z1_MODEL_PATH}")
    model = pin.buildModelFromMJCF(Z1_MODEL_PATH)
    data = model.createData()

    if model.nv != len(MODELED_DDS_INDICES):
        print(f"[错误] 模型自由度 ({model.nv}) 与您定义的已建模关节数量 ({len(MODELED_DDS_INDICES)}) 不匹配！")
        return
        
    print(f"模型加载成功，包含 {model.nv} 个自由度。")
    # print(f"非线性阻抗控制关节 (手臂): {GRAV_COMP_INDICES}")
    # print(f"零位保持关节 (其他): {ZERO_POS_INDICES}")
    
    dds_to_pin_idx_map = {dds_idx: pin_idx for pin_idx, dds_idx in enumerate(MODELED_DDS_INDICES)}
    
    q_all = np.zeros(NUM_MOTORS)
    v_all = np.zeros(NUM_MOTORS)

    print("\n启动混合控制循环（手臂非线性阻抗 + 其余关节零位保持）...")
    # print("所有关节均使用力位混合控制 (mode=0)。")
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
                    # --- 手臂关节：应用重力补偿 + 非线性阻抗 ---
                    cmd.kp = ARM_KP
                    
                    # --- 核心修改：计算并应用非线性阻尼 ---
                    v_current = v_all[i]
                    v_desired = 0.0
                    v_error = v_desired - v_current
                    
                    # 添加一个极小值epsilon避免v_error为0时计算错误
                    epsilon = 1e-5
                    
                    # 计算非线性阻尼系数 kd_nonlinear = μ * |v_error|^(n-1)
                    kd_nonlinear = NONLINEAR_MU * (np.abs(v_error) + epsilon)**(NONLINEAR_N - 1)
                    cmd.kd = kd_nonlinear
                    # --- 核心修改结束 ---

                    cmd.pos = q_all[i]
                    cmd.vel = v_desired
                    
                    pin_idx = dds_to_pin_idx_map[i]
                    # cmd.tau = tau_gravity_24[pin_idx]
                    cmd.tau = 0

                else:
                    # --- 其他关节：应用PD控制以保持零位 ---
                    cmd.kp = ZERO_POS_KP
                    cmd.kd = ZERO_POS_KD
                    cmd.pos = 0.0
                    cmd.vel = 0.0
                    cmd.tau = 0.0

            z1_robot.setCommand()

            # 稳定控制频率
            elapsed = time.time() - loop_start_time
            sleep_time = 0.004 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            n = n + 1
            if n > 100:
                print(tau_gravity_24)
                n = 1

    except KeyboardInterrupt:
        print("\n正在停止混合控制循环。")
    finally:
        print("正在关闭Z1远程客户端。")
        z1_robot.stop()

if __name__ == '__main__':
    main()