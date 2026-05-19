# YKS_SDK_WB 修复计划（48 槽位 / 实际 29 电机）

> 背景：项目在 commit `2329494 (canfd功能增加)` → `c7f5024 (拓展到48位)` 后，从站槽位扩到 2×24=48。
> 但实际只用了 **29 个电机**（左腿 6 + 右腿 6 + 腰 3 + 左臂 7 + 右臂 7），存在编号间隙。
> 配套配置（电机类型、运动学、终端打印、DDS、Python 客户端）还没跟上。
>
> 真值来源：
> - 电机分布与型号：[example/Z2.0-Lite电机编号.docx](../example/Z2.0-Lite电机编号.docx)
> - 电机参数（KT / KD / 转矩区间 / 电流区间）：[example/ENCOS+电机数据手册+V3.13 (1).pdf](../example/ENCOS+电机数据手册+V3.13%20(1).pdf)

## 0. 设计决策（已与用户确认）

1. **arm 14 个电机**：包含 docx 括号内的小臂侧摆 + 腕周转 + 腕俯仰，单臂 7 个。
2. **新增电机型号**：从 ENCOS PDF 查 `A4315 / A8116 / A6416 / A2806` 的规格，写新枚举与新宏；不做近似复用。
3. **保留 ID 间隙**：底层数组仍按 48 容量，`global_id` 与 docx 上 "motor#" 对齐（motor#1→id=0，motor#9→id=8…）。打印 / DDS 通道按"活跃列表"过滤。

## 1. 活跃电机清单（核心常量，多处复用）

新建 [robot_layout.h](robot_layout.h)（C 头，C/C++ 可共用），集中导出：

```c
#define LEG_MOTOR_NUMBER     12
#define BODY_MOTOR_NUMBER     3
#define ARM_MOTOR_NUMBER     14
#define ACTIVE_MOTOR_NUMBER  (LEG_MOTOR_NUMBER + BODY_MOTOR_NUMBER + ARM_MOTOR_NUMBER) /* 29 */

extern const int kLegMotorIds[LEG_MOTOR_NUMBER];   /* {0,1,2,3,4,5, 8,9,10,11,12,13} */
extern const int kBodyMotorIds[BODY_MOTOR_NUMBER]; /* {16,17,18} */
extern const int kArmMotorIds[ARM_MOTOR_NUMBER];   /* {24,25,26,27,28,29,30, 32,33,34,35,36,37,38} */
extern const int kAllActiveIds[ACTIVE_MOTOR_NUMBER];

/* 0-based 在 DDS 紧凑数组里的下标 → global_id 反查 */
int leg_local_to_global(int local);
int body_local_to_global(int local);
int arm_local_to_global(int local);
/* global_id → DDS 紧凑下标，找不到返回 -1 */
int leg_global_to_local(int global_id);
int body_global_to_local(int global_id);
int arm_global_to_local(int global_id);

/* 关节人类可读名（用于终端打印） */
extern const char *kJointNames[TOTAL_MOTOR_NUMBER]; /* 未占用槽位填 "" */
```

把 `LEG_MOTOR_NUMBER / ARM_MOTOR_NUMBER / BODY_MOTOR_NUMBER` 从 [transmit.h:11-13](transmit.h) **删除**，统一从 `robot_layout.h` 取，避免双定义。`TOTAL_MOTOR_NUMBER = 48` 保留在 transmit.h。

> **注意**：当前 `BODY_MOTOR_NUMBER` 是 6，要改成 3。`ARM_MOTOR_NUMBER` 当前是 12，改成 14。

## 2. 电机型号扩展（步骤 1：先查 PDF）

任务：codex 打开 [ENCOS+电机数据手册+V3.13 (1).pdf](../example/ENCOS+电机数据手册+V3.13%20(1).pdf)，对下列四个新型号查 **额定/峰值力矩 (Nm)、额定/峰值电流 (A)、力矩常数 KT、KD 适用范围**：

| 新型号             | 用途           | 在哪些 global_id   |
| ------------------ | -------------- | ------------------ |
| EC-A4315-P2-36     | 踝、腰侧摆俯仰 | 4,5,12,13,16,17    |
| EC-A8116-P1-18H    | 髋俯仰、膝     | 0,3,8,11           |
| EC-A6416-P2-30.25H | 髋侧摆         | 1,9                |
| EC-A2806-P2-36     | 腕周转俯仰     | 29,30,37,38        |

在 [motor_control.h](motor_control.h) 添加：

```c
//EC-A4315-P2-36     —— 从 PDF 查
#define KT4315 ?.??f
#define T4315_MIN (-??.0f)
#define I4315_MIN (-??.0f)
#define T4315_MAX  ??.0f
#define I4315_MAX  ??.0f

//EC-A8116-P1-18H
#define KT8116 ?.??f
#define T8116_MIN (-??.0f)
#define I8116_MIN (-??.0f)
#define T8116_MAX  ??.0f
#define I8116_MAX  ??.0f

//EC-A6416-P2-30.25H
#define KT6416 ?.??f
#define T6416_MIN (-??.0f)
#define I6416_MIN (-??.0f)
#define T6416_MAX  ??.0f
#define I6416_MAX  ??.0f

//EC-A2806-P2-36
#define KT2806 ?.??f
#define T2806_MIN (-??.0f)
#define I2806_MIN (-??.0f)
#define T2806_MAX  ??.0f
#define I2806_MAX  ??.0f
```

把 `enum YKS_MOTOR_TYPE` 扩为 11 项，**追加在末尾**（不要插中间，避免破坏 `Z1_MOTOR_ID_Type[]` 旧值的解读）：

```c
enum YKS_MOTOR_TYPE {
    A4310 = 0, A6408 = 1, A8112 = 2,
    A10020_1 = 3, A10020_2 = 4, A13715 = 5, A13720 = 6,
    A4315 = 7, A8116 = 8, A6416 = 9, A2806 = 10
};
```

并把 `YKS_MOTOR_RANGE` 的每个数组从 7 项扩到 **11 项**，按 enum 顺序追加 KD/T/I/KT。`KD_MIN/MAX` 暂时沿用 `KD1_*`/`KD2_*`（按机型类似度归属，PDF 没明确就保守用 `KD1_*`）。

> 同步修改 [motor_control.c:3-11](motor_control.c) 的 `yks_motor_range` 初始化列表。

## 3. `Z1_MOTOR_ID_Type[]` 重写（[motor_control.c:25-34](motor_control.c#L25-L34)）

按 docx 一一对齐到 global_id：

```c
int Z1_MOTOR_ID_Type[TOTAL_MOTOR_NUMBER] = {
    /* 0-5  左腿 */
    A8116, A6416, A6408, A8116, A4315, A4315,
    /* 6-7  预留（空槽，type 任意，建议放 A8112 占位） */
    A8112, A8112,
    /* 8-13 右腿 */
    A8116, A6416, A6408, A8116, A4315, A4315,
    /* 14-15 预留 */
    A8112, A8112,
    /* 16-18 腰：侧摆/俯仰/回转 */
    A4315, A4315, A8112,
    /* 19-23 预留 */
    A8112, A8112, A8112, A8112, A8112,
    /* 24-30 左臂：肩俯仰/肩侧摆/肩周转/肘/小臂侧摆/腕周转/腕俯仰 */
    A6408, A4310, A4310, A4310, A4310, A2806, A2806,
    /* 31 预留 */
    A8112,
    /* 32-38 右臂 */
    A6408, A4310, A4310, A4310, A4310, A2806, A2806,
    /* 39-47 预留 */
    A8112, A8112, A8112, A8112, A8112, A8112, A8112, A8112, A8112
};
```

> docx 里 "肩部俯仰"对应 1 个，"肩部侧摆与周转"对应 **2 个** (motor#26,27 / #34,35)，"肘"1 个，"小臂侧摆"1 个，"腕部周转与俯仰" 2 个 (motor#30,31 / #38,39)。合计单臂 7 个，**与本表完全一致**。

## 4. 力矩 / 速度 / 位置上下限数组（[transmit.cpp:43-103](transmit.cpp#L43-L103)）

`Z1_MOTOR_TOR_MAX / MIN` 改成"按 global_id 索引取该机型 max"。要么按真实型号显式列 48 项，要么直接在初始化时从 `yks_motor_range` 派生：

```cpp
float Z1_MOTOR_TOR_MAX[TOTAL_CAN_NUMBER];
float Z1_MOTOR_TOR_MIN[TOTAL_CAN_NUMBER];

__attribute__((constructor)) static void init_motor_limits() {
    for (int i = 0; i < TOTAL_MOTOR_NUMBER; ++i) {
        int t = Z1_MOTOR_ID_Type[i];
        Z1_MOTOR_TOR_MAX[i] = yks_motor_range.T_MAX[t];
        Z1_MOTOR_TOR_MIN[i] = yks_motor_range.T_MIN[t];
    }
}
```

（或者保持现在的显式数组写法，但要按 Z1_MOTOR_ID_Type 的新分布对齐：左腿 `T8116_MAX, T6416_MAX, T6408_MAX, T8116_MAX, T4315_MAX, T4315_MAX` 之类。`__attribute__((constructor))` 方案不易出错，推荐。）

`Z1_MOTOR_POS_MAX / MIN`、`Z1_MOTOR_SPE_MAX / MIN` 保持现有的 `POS_LEG_MECH_*` / `POS_MAX` / `SPD_MAX` 的"按部位粗分"是可以的，但要按新的活跃分布写满 48 项。**未使用槽位填 0/0**（避免误下发到空槽）。

## 5. [Z1Legs.h](../Z1Legs.h) 关节枚举与方向数组

替换 `enum Z1JointIndex` 为稀疏布局：

```cpp
enum Z1JointIndex {
    // 左腿
    LeftHipPitch    = 0,
    LeftHipRoll     = 1,
    LeftHipYaw      = 2,
    LeftKnee        = 3,
    LeftAnklePitch  = 4,  LeftAnkleB = 4,
    LeftAnkleRoll   = 5,  LeftAnkleA = 5,
    // 右腿
    RightHipPitch   = 8,
    RightHipRoll    = 9,
    RightHipYaw     = 10,
    RightKnee       = 11,
    RightAnklePitch = 12, RightAnkleB = 12,
    RightAnkleRoll  = 13, RightAnkleA = 13,
    // 腰
    WaistRoll       = 16,
    WaistPitch      = 17,
    WaistYaw        = 18,
    // 左臂
    LeftShoulderPitch = 24,
    LeftShoulderRoll  = 25,
    LeftShoulderYaw   = 26,
    LeftElbow         = 27,
    LeftForearmRoll   = 28,
    LeftWristYaw      = 29,
    LeftWristPitch    = 30,
    // 右臂
    RightShoulderPitch = 32,
    RightShoulderRoll  = 33,
    RightShoulderYaw   = 34,
    RightElbow         = 35,
    RightForearmRoll   = 36,
    RightWristYaw      = 37,
    RightWristPitch    = 38,
};
```

> **重要**：原代码里 `RightAnklePitch = 10` 在新分布下要改成 12。`Z1Legs.cpp` 里 `setMotorCommand / getMotorData` 的 `LeftAnkleA / LeftAnkleB / RightAnkleA / RightAnkleB` 分支会自动跟着 enum 更新，**不需要改函数主体**，但要核对所有引用。

`LegDirectionMotor_[TOTAL_MOTOR_NUMBER]` 要改成 48 项；非活跃槽位填 `1`（无所谓）。**活跃槽位的正负号要按机械装配实测填**——保留计划里的占位 `1`，由现场调试时改：

```cpp
const int LegDirectionMotor_[TOTAL_MOTOR_NUMBER] = {
    // 0-5 左腿
    -1, 1, -1, -1, -1, 1,
    // 6-7 预留
    1, 1,
    // 8-13 右腿
    -1, 1, -1, 1, -1, 1,
    // 14-15 预留
    1, 1,
    // 16-18 腰
    1, 1, 1,
    // 19-23 预留
    1, 1, 1, 1, 1,
    // 24-30 左臂
    1, 1, 1, 1, 1, 1, 1,
    // 31 预留
    1,
    // 32-38 右臂
    1, 1, 1, 1, 1, 1, 1,
    // 39-47 预留
    1, 1, 1, 1, 1, 1, 1, 1, 1
};
```

`Kp / Kd` 数组同步扩到 48 项的全 0（保持现有行为）。

## 6. [Z1Legs.cpp](../Z1Legs.cpp) 终端只显示活跃电机

修改 `PrintMotorState`：用 `kAllActiveIds[]` 列表打印，每行带关节名。

```cpp
void Z1Legs::PrintMotorState(int /*size*/) const {
    // ... 顶部 flag 行不变 ...
    mvprintw(4, 0,  "ID  | Joint               | Pos      | Vel      | Tau      | DesPos   | DesVel   | KP    | KD    | FF");
    mvprintw(5, 0,  "-------------------------------------------------------------------------------------------------------------");

    for (int row = 0; row < ACTIVE_MOTOR_NUMBER; ++row) {
        int i = kAllActiveIds[row];
        move(row + 6, 0);
        clrtoeol();
        mvprintw(row + 6, 0,  "%2d", i);
        mvprintw(row + 6, 6,  "%-20s", kJointNames[i]);
        mvprintw(row + 6, 28, "%9.3f", finiteOrZero(motor_print_[i].pos_));
        mvprintw(row + 6, 39, "%9.3f", finiteOrZero(motor_print_[i].vel_));
        mvprintw(row + 6, 50, "%9.3f", finiteOrZero(motor_print_[i].tau_));
        mvprintw(row + 6, 61, "%9.3f", finiteOrZero(motor_print_[i].pos_des_));
        mvprintw(row + 6, 72, "%9.3f", finiteOrZero(motor_print_[i].vel_des_));
        mvprintw(row + 6, 83, "%7.3f", finiteOrZero(motor_print_[i].kp_));
        mvprintw(row + 6, 91, "%7.3f", finiteOrZero(motor_print_[i].kd_));
        mvprintw(row + 6, 99, "%7.3f", finiteOrZero(motor_print_[i].ff_));
    }
}
```

`kJointNames[]` 在 `robot_layout.c` 里填好："LeftHipPitch" / "LeftHipRoll" / ... / "RightWristPitch"，未使用槽位填空串（不会被遍历到，仅作占位）。

## 7. DDS 通道压缩到 3 个（arm / leg / body）

### 7.1 [main.cpp](../main.cpp) 删除 WHOLEBODY 通道

- 删除：`#define WHOLEBODYCMDTOPIC` / `#define WHOLEBODYSTATETOPIC` 及对应 topic/reader/writer 创建块（[main.cpp:30-31](../main.cpp#L30-L31), [main.cpp:450-462](../main.cpp#L450-L462), [main.cpp:560-571](../main.cpp#L560-L571)）。
- 删除：`DDS_Z1_5_WB_SUB / DDS_Get_Z1_5_WB_Motor_Cmds / DDS_Pub_Z1_5_WB_Motor_Data` 三个函数 + 主循环里的调用 ([main.cpp:295-306, 120-131, 212-231, 681](../main.cpp))。
- [DDSRelate.h](../DDSRelate.h) / [DDSRelate.cpp](../DDSRelate.cpp) 同步删除 `DDS_Get_Z1_5_WB_Motor_Cmds / DDS_Pub_Z1_5_WB_Motor_Data`。

### 7.2 Pub / Sub 改成稀疏映射（关键）

把原来"i+12 / i+24"那种连续偏移改成查表：

```cpp
// 收：DDS index i 在 cmds 数组里 → 写到 motor_data_[kArmMotorIds[i]]
void DDS_Get_Arm_Motor_Cmds(const motorcmds &cmds, YKSMotorData *motor_cmds) {
    const int n = std::min<int>(ARM_MOTOR_NUMBER, (int)cmds.cmds().size());
    for (int i = 0; i < n; ++i) {
        int gid = kArmMotorIds[i];
        motor_cmds[gid].pos_des_ = cmds.cmds()[i].pos();
        motor_cmds[gid].vel_des_ = cmds.cmds()[i].vel();
        motor_cmds[gid].ff_      = cmds.cmds()[i].tau();
        motor_cmds[gid].mode     = cmds.cmds()[i].mode();
        motor_cmds[gid].kp_      = cmds.cmds()[i].kp();
        motor_cmds[gid].kd_      = cmds.cmds()[i].kd();
    }
}
// 发：紧凑数组的第 i 个 = motor_data_[kArmMotorIds[i]]
void DDS_Pub_Arm_Motor_Data(motorstates &states, dds::pub::DataWriter<motorstates> &writer,
                            const YKSMotorData *motor_data_) {
    for (int i = 0; i < ARM_MOTOR_NUMBER; ++i) {
        int gid = kArmMotorIds[i];
        auto &s = states.states()[i];
        s.mode(motor_data_[gid].mode);
        s.index(gid);                       // <-- 真实 global_id，订阅方据此区分
        s.pos(motor_data_[gid].pos_);
        s.vel(motor_data_[gid].vel_);
        s.cur(motor_data_[gid].tau_);
        s.tau(motor_data_[gid].tau_);
        s.tau_raw(motor_data_[gid].tau_);
        s.error(motor_data_[gid].error_);
        s.tem(motor_data_[gid].temperature_);
        s.mos_tem(motor_data_[gid].mos_temperature_);
    }
    writer.write(states);
}
```

leg / body 同理，分别用 `kLegMotorIds[]`、`kBodyMotorIds[]`。**不再有"+12"、"+24"硬编码偏移**。

### 7.3 `motorstates` 数组尺寸

在 [main.cpp:607-621](../main.cpp#L607-L621):
```cpp
armStates.states().resize(ARM_MOTOR_NUMBER);   // 14
legStates.states().resize(LEG_MOTOR_NUMBER);   // 12
bodyStates.states().resize(BODY_MOTOR_NUMBER); // 3
// 删除 z1_5_wb_States
```

### 7.4 `level` 字段约定

| 通道 | level |
| ---- | ----- |
| LEG  | 0     |
| ARM  | 1     |
| BODY | 2     |

> Z1_5_WB 原来用 level=3，本次删除后该值不再使用。

## 8. Python 客户端 [example/yksddss/yksddspy/z1_three.py](../example/yksddss/yksddspy/z1_three.py)

### 8.1 删除 WHOLEBODY 相关常量与逻辑（如果有），保持只有 3 个角色。

### 8.2 修改 motornum / levels：

```python
motornum = {'arm': 14, 'leg': 12, 'body': 3}
levels   = {'leg': 0,  'arm':  1, 'body': 2}
```

### 8.3 加映射表 + 改 setter

```python
LEG_GIDS  = [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13]
BODY_GIDS = [16, 17, 18]
ARM_GIDS  = [24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38]

def leg_squat_control(self, leg_index, mode, pos, vel, tau, kp, kd):
    """leg_index 是 global_id（0..5 或 8..13）"""
    if leg_index not in LEG_GIDS:
        raise IndexError(f"Invalid leg motor global_id: {leg_index}")
    local = LEG_GIDS.index(leg_index)
    cmd = self.motorCmds.cmds[local]
    cmd.mode, cmd.pos, cmd.vel, cmd.tau, cmd.kp, cmd.kd = mode, pos, vel, tau, kp, kd

def arm_yks_squat_control(self, arm_index, mode, pos, vel, tau, kp, kd):
    """arm_index 是 global_id（24..30 或 32..38）"""
    if arm_index not in ARM_GIDS:
        raise IndexError(f"Invalid arm motor global_id: {arm_index}")
    local = ARM_GIDS.index(arm_index)
    cmd = self.motorCmds.cmds[local]
    cmd.mode, cmd.pos, cmd.vel, cmd.tau, cmd.kp, cmd.kd = mode, pos, vel, tau, kp, kd

def body_yks_squat_control(self, body_index, mode, pos, vel, tau, kp, kd):
    """body_index 是 global_id（16..18）"""
    if body_index not in BODY_GIDS:
        raise IndexError(f"Invalid body motor global_id: {body_index}")
    local = BODY_GIDS.index(body_index)
    cmd = self.motorCmds.cmds[local]
    cmd.mode, cmd.pos, cmd.vel, cmd.tau, cmd.kp, cmd.kd = mode, pos, vel, tau, kp, kd

def read_arm_ti5_control(self, arm_index, mode):
    if arm_index not in ARM_GIDS:
        raise IndexError("Invalid motor index")
    local = ARM_GIDS.index(arm_index)
    state = self._motorStates.states[local]
    return {1: state.pos, 2: state.tau, 3: state.vel}.get(mode, 0)
```

### 8.4 `__main__` 段示例

把 `for i in range(12): z1_leg.leg_squat_control(i, ...)` 这种"0..11 连续"的循环改成遍历 `LEG_GIDS`：

```python
for gid in LEG_GIDS:
    z1_leg.leg_squat_control(gid, 0, 0, 0, 0, 100, 10)
```

`joint_angles / ksp_stance / ksd_stance` 这些 12 项数组的下标仍按"紧凑下标 0..11"对应 `LEG_GIDS[i]`，配合上面 `local = LEG_GIDS.index(gid)` 一致即可。

## 9. 通讯通路自检清单（codex 完成后照着跑）

1. **编译**
   - `cd YKS_SDK_WB/cmake-build-debug && cmake .. && make -j` 通过；无 `LEG_MOTOR_NUMBER`/`ARM_MOTOR_NUMBER`/`BODY_MOTOR_NUMBER` 重定义警告。
2. **常量一致性**
   - `grep -n "LEG_MOTOR_NUMBER\|ARM_MOTOR_NUMBER\|BODY_MOTOR_NUMBER" -r .` 全部来自 `robot_layout.h`，没有残留 `12/12/6` 硬编码。
   - `grep -n "i + 12\|i + 24" -r YKS_SDK_WB/*.cpp` 0 命中（旧的偏移已经全换成查表）。
   - `grep -n "WHOLEBODY\|Z1_5_WB\|wholebodymotor" -r .` 0 命中（已彻底清理）。
3. **运行行为**
   - 终端不再有 48 行空显示，**恰好 29 行**，每行带 Joint 名字。
   - `main.cpp` 主循环只订阅/发布 leg / arm / body 三个 topic（cyclonedds-cli `dds_probe` 验证）。
4. **DDS payload 验证**
   - 启动 `python3 z1_three.py`，调用 `leg_squat_control(8, 0, 0.1, ...)`（右髋俯仰）→ 在 SDK 端用 `printf("[motor 8] pos_des=%.3f", motor_data_[8].pos_des_)` 应看到 0.1。
   - 反向：SDK 端给 `motor_data_[18].pos_ = 0.5`（腰回转） → python `getStates().states[2].pos` 应得 0.5（local idx 2 = global 18）。
5. **极限保护**
   - 给 `motor_data_[16].ff_ = 9999`（腰侧摆 A4315），下发后应被截到 `T4315_MAX`，不能再被 `T8112_MAX` 错截。

## 10. 修改文件清单（codex checklist）

| 文件                                                         | 改动类型 |
| ------------------------------------------------------------ | -------- |
| `YKS_SDK_WB/app/robot_layout.h` / `robot_layout.c`           | **新建** |
| `YKS_SDK_WB/app/transmit.h`                                  | 删除 LEG/ARM/BODY 宏 |
| `YKS_SDK_WB/app/transmit.cpp`                                | 改 Z1_MOTOR_*_MAX/MIN 数组按新分布 |
| `YKS_SDK_WB/app/motor_control.h`                             | 新增 KT4315/KT8116/KT6416/KT2806 等宏；扩 enum 与 YKS_MOTOR_RANGE |
| `YKS_SDK_WB/app/motor_control.c`                             | `yks_motor_range` 数组扩 11 项；重写 `Z1_MOTOR_ID_Type[]` |
| `YKS_SDK_WB/app/CMakeLists.txt`                              | 把新文件加进 add_library |
| `YKS_SDK_WB/Z1Legs.h`                                        | 重写 `enum Z1JointIndex`；扩 LegDirectionMotor_ / Kp / Kd 到 48 项 |
| `YKS_SDK_WB/Z1Legs.cpp`                                      | 改 `PrintMotorState` 用 kAllActiveIds + kJointNames |
| `YKS_SDK_WB/main.cpp`                                        | 删 WHOLEBODY；pub/sub 改用 kLegMotorIds/kBodyMotorIds/kArmMotorIds |
| `YKS_SDK_WB/DDSRelate.h` / `DDSRelate.cpp`                   | 同步删 Z1_5_WB 函数；剩余三个改稀疏映射 |
| `YKS_SDK_WB/example/yksddss/yksddspy/z1_three.py`            | 改 motornum/levels；setter 改用 *_GIDS 反查 |

## 11. 不需要改的部分（明确划界）

- `app/motor_control.c` 里 `RV_can_data_repack` / `send_motor_ctrl_cmd` / `EtherCAT_Send_Command` 这些底层收发函数已经按 `g_motor_map[i].slave_idx / pdo_slot / can_id / global_id` 工作了（commit `2329494` 引入），**不需要再改**。它对 48 槽位的稀疏使用是天然支持的——未配置的槽位 `mot_data[i].mode == 0` 且 kp/kd/pos_des/vel_des/ff = 0，下发的就是"零控制"帧，不影响。
- `g_motor_map[] / g_slaves[]` 当前是 1:1 全填的 48 项，**保留**——非活跃槽位下发零帧无害。如果担心总线流量，可在第 6 步以后再做"只对 active 槽位调用 send"的优化，本次先不动。
- BMS / IMU / SBUS / Logic / Hcmd 这几个 DDS 通道与本次改动无关，**保持不变**。
- `command.cpp` / `command.h` 调试命令保持不变（slaveId 0-based、pdo_slot 1-24 仍然对的）。

## 12. 执行顺序建议

1. 先建 `robot_layout.h/.c`（步骤 1），让全项目能 include。
2. 改 `transmit.h` 删宏（步骤 1 末段）。
3. PDF 查参数 → `motor_control.h/.c`（步骤 2、3、4）。
4. Z1Legs 头与实现（步骤 5、6）。
5. main + DDSRelate（步骤 7）。
6. python 客户端（步骤 8）。
7. 跑步骤 9 的自检。

按这个顺序，每步都能独立编译过——避免一次性巨改之后定位编译错误。
