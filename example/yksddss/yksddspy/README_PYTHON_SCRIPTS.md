# Python DDS SDK 使用说明

本目录现在推荐把 [z2_sdk.py](z2_sdk.py) 作为对外提供的 Python SDK。  
它把下肢、上肢、躯干、全身 4 组 DDS command/state 话题统一到一个文件里，外部用户只需要导入这个 SDK，就可以收电机状态、发电机指令，或参考示例做慢速到位控制。

---

## 1. 文件说明

```text
yksddss/yksddspy/
├── z2_sdk.py                # 推荐对外使用的统一 DDS Python SDK
├── z2_wholebody_low.py      # 慢速到达目标位置示例，继承 z2_sdk.Z2WholeBodyClient
└── README_PYTHON_SCRIPTS.md # 本说明文档
```

外部用户主要看：

1. [z2_sdk.py](z2_sdk.py)：SDK 本体。
2. [z2_wholebody_low.py](z2_wholebody_low.py)：如何继承 SDK 并做缓慢插值移动。

---

## 2. DDS 话题

`z2_sdk.py` 内置了 4 组 command/state 话题：

| 控制组 | 指令话题 | 状态话题 | level | 电机数 |
|---|---|---|---:|---:|
| leg | `/nubot/z1/legmotorcmds` | `/nubot/z1/legmotorstates` | 0 | 12 |
| arm | `/nubot/z1/armmotorcmds` | `/nubot/z1/armmotorstates` | 1 | 14 |
| body | `/nubot/z1/bodymotorcmds` | `/nubot/z1/bodymotorstates` | 2 | 3 |
| wholebody | `/nubot/z1/wholebodymotorcmds` | `/nubot/z1/wholebodymotorstates` | 3 | 29 |

如果只是给用户一个最简单、最完整的例子，建议使用 wholebody：

```python
from z2_sdk import Z2WholeBodyClient

client = Z2WholeBodyClient()
```

wholebody 内部使用：

```python
WHOLEBODYCMDTOPIC = "/nubot/z1/wholebodymotorcmds"
WHOLEBODYSTATETOPIC = "/nubot/z1/wholebodymotorstates"
```

---

## 3. 电机 ID 顺序

`z2_sdk.py` 使用全局电机 ID，不使用连续本地编号。全身顺序必须和 C++ 端 `kAllActiveIds` 一致：

```python
LEG_GIDS = [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13]
BODY_GIDS = [16, 17, 18]
ARM_GIDS = [24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38]
ALL_GIDS = LEG_GIDS + BODY_GIDS + ARM_GIDS
```

全身目标位置数组 `WHOLEBODY_TARGET_STANCE` 每一行格式为：

```python
(global_id, target_pos, kp, kd)
```

含义：

| 字段 | 单位 | 说明 |
|---|---|---|
| `global_id` | - | 电机全局 ID，必须按 `ALL_GIDS` 顺序排列 |
| `target_pos` | rad | 目标关节位置 |
| `kp` | - | 位置刚度 |
| `kd` | - | 阻尼 |

`build_command_maps()` 会检查 `WHOLEBODY_TARGET_STANCE` 的 ID 顺序。如果顺序不对，会直接抛错，避免把目标位置发给错误电机。

---

## 4. 直接运行 SDK：一次收发示例

`z2_sdk.py` 可以直接运行。SDK 内部会启动后台线程持续收发 DDS；这里的“一次收发”指用户层只调用一次读状态、一次更新命令快照。它会：

1. 创建 `Z2WholeBodyClient`
2. 从后台缓存中读取一次 `/nubot/z1/wholebodymotorstates` 的 `motorstates`
3. 打印这一帧状态
4. 按 `WHOLEBODY_TARGET_STANCE` 生成一帧 `motorcmds`
5. 更新一次 `/nubot/z1/wholebodymotorcmds` 的命令快照
6. 退出

运行：

```bash
cd /home/nubot/Project/humanoid_proj0605/humanoid_proj/z1_rl/YKS_SDK_WB/example/yksddss/yksddspy
python3 z2_sdk.py
```

用户层调用 `set_motor()` 修改 `client.motorCmds`，再调用 `setCommand()` 更新一次发送快照；后台线程会按默认 2ms 周期持续发布最新快照，直到 `client.stop()`。

---

## 5. 作为库使用

### 5.1 全身收一次状态

```python
from z2_sdk import Z2WholeBodyClient, print_states, wait_for_current_positions

client = Z2WholeBodyClient()
wait_for_current_positions(client, timeout=5.0)
states = client.getStates()
print_states(states)
```

`getStates()` 和 `z1_wholebody.py` 一样，是读取后台线程最近一次收到的缓存状态；它本身不直接调用 DDS `take()`。

`states.states[i]` 中常用字段：

| 字段 | 单位 | 说明 |
|---|---|---|
| `index` | - | 电机全局 ID |
| `pos` | rad | 当前位置 |
| `vel` | rad/s | 当前速度 |
| `tau` | Nm | 当前力矩 |
| `cur` | A | 当前电流 |
| `tem` | degC | 电机温度 |
| `mos_tem` | degC | MOS 温度 |
| `error` | - | 错误码 |

### 5.2 全身读取当前位置并校验 ID

```python
from z2_sdk import Z2WholeBodyClient, print_positions, wait_for_current_positions

client = Z2WholeBodyClient()
positions = wait_for_current_positions(client, timeout=5.0)
print_positions(positions, title="current wholebody positions")
```

`wait_for_current_positions()` 会检查：

- 是否在超时时间内收到状态
- 状态数组长度是否足够
- `state.index` 是否和 `ALL_GIDS` 顺序一致
- 位置是否为有限数值，拒绝 NaN/Inf

### 5.3 全身发一次目标指令

```python
from z2_sdk import (
    WHOLEBODY_TARGET_STANCE,
    Z2WholeBodyClient,
    build_command_maps,
)

client = Z2WholeBodyClient()
target_positions, kps, kds = build_command_maps(WHOLEBODY_TARGET_STANCE)

for local, gid in enumerate(client.gids):
    client.set_motor(
        gid,
        mode=0,
        pos=target_positions[gid],
        kp=kps[gid],
        kd=kds[gid],
    )

client.setCommand()
```

`set_motor()` 只修改外部缓冲区 `motorCmds`；`setCommand()` 把完整一帧快照同步给后台发送缓冲区，后台线程会持续发布该快照。

默认 `mode=0` 是位置控制。常用约定：

| mode | 含义 |
|---:|---|
| 0 | 位置控制 |
| 1 | 速度控制 |
| 2 | 力矩控制 |

### 5.4 持续保持目标位置

```python
import time

from z2_sdk import (
    WHOLEBODY_TARGET_STANCE,
    Z2WholeBodyClient,
    build_command_maps,
)

client = Z2WholeBodyClient()
target_positions, kps, kds = build_command_maps(WHOLEBODY_TARGET_STANCE)

for gid in client.gids:
    client.set_motor(gid, mode=0, pos=target_positions[gid], kp=kps[gid], kd=kds[gid])
client.setCommand()

while True:
    time.sleep(1.0)
```

上面只需要调用一次 `setCommand()`。后台线程会持续发布最新命令快照；循环只是让 Python 进程保持运行。

---

## 6. 慢速到达目标位置示例

[z2_wholebody_low.py](z2_wholebody_low.py) 展示了如何继承 `Z2WholeBodyClient`，先读取当前位置，再缓慢插值到 `WHOLEBODY_TARGET_STANCE`。

运行：

```bash
cd /home/nubot/Project/humanoid_proj0605/humanoid_proj/z1_rl/YKS_SDK_WB/example/yksddss/yksddspy
python3 z2_wholebody_low.py
```

流程：

```text
1. 初始化 Z2WholeBodyLowClient
2. wait_for_current_positions(timeout=5.0) 读取并校验当前全身位置
3. 保持当前位置 0.5s
4. 用 10s 从当前位置线性插值到 WHOLEBODY_TARGET_STANCE
5. 到达后保持进程运行，后台线程持续发布目标位置，直到 Ctrl+C
```

核心代码：

```python
class Z2WholeBodyLowClient(Z2WholeBodyClient):
    def ramp_to_targets(self, start_positions, target_positions, kps, kds,
                        duration=10.0, period=0.02):
        steps = max(1, int(duration / period))
        for step in range(steps + 1):
            alpha = step / steps
            positions = interpolate_positions(ALL_GIDS, start_positions, target_positions, alpha)
            for gid in ALL_GIDS:
                self.set_motor(gid, mode=0, pos=positions[gid], kp=kps[gid], kd=kds[gid])
            self.setCommand()
            time.sleep(period)
```

用户只需要修改 [z2_sdk.py](z2_sdk.py) 里的 `WHOLEBODY_TARGET_STANCE`，就可以改变目标姿态。

---

## 7. 只控制 leg / body / arm

如果用户不想用 wholebody，也可以使用通用类 `Z2MotorClient`：

```python
from z2_sdk import Z2MotorClient

leg_client = Z2MotorClient(role="leg")
body_client = Z2MotorClient(role="body")
arm_client = Z2MotorClient(role="arm")
```

每个 role 会自动选择对应 topic、level 和 GID 顺序。比如 `role="leg"` 只会收发 12 个下肢电机。

示例：读取下肢当前位置。

```python
from z2_sdk import Z2MotorClient, print_positions, wait_for_current_positions

leg_client = Z2MotorClient(role="leg")
leg_positions = wait_for_current_positions(leg_client, timeout=5.0)
print_positions(leg_positions, title="leg positions")
```

---

## 8. 运行前检查

运行 Python 脚本前请确认：

1. C++ 端 `main.cpp` 已运行，并且创建了对应 DDS reader/writer。
2. Python 和 C++ 使用同一个 DDS domain，当前代码默认 `domain_id=0`。
3. Python 环境能导入 `nubotddsmsg.hr`。
4. `CYCLONEDDS_URI` 指向正确网络配置，Python 和 C++ 在同一 DDS 网络内。
5. wholebody 状态话题中每帧应包含 29 个状态，顺序应与 `ALL_GIDS` 一致。

如果报：

```text
ModuleNotFoundError: No module named 'nubotddsmsg'
```

说明 Python 消息包没有在当前环境中。需要先生成或配置 `nubotddsmsg` 的 Python 包路径。

如果 `wait_for_current_positions()` 超时，优先检查：

- C++ 端是否正在运行
- DDS 网络配置是否正确
- topic 名称是否一致
- `main.cpp` 是否正在发布 `/nubot/z1/wholebodymotorstates`

---

## 9. 安全注意事项

- 目标位置单位是 rad，不是角度。
- 修改 `WHOLEBODY_TARGET_STANCE` 前确认每个 `global_id` 对应的关节。
- 不要删除或打乱 `WHOLEBODY_TARGET_STANCE` 的 ID 顺序。
- 第一次调试建议先把目标位置设为当前安全姿态附近的小幅变化。
- `z2_sdk.py` 后台会持续发布最新命令快照，但不会自动急停。
- 用户层调用 `setCommand()` 一次后，命令会持续生效到下一次更新或 `client.stop()`。
- 需要缓慢移动时，参考 `z2_wholebody_low.py`，不要直接从大角度跳变到目标姿态。

---

## 10. 常用 API 速查

| API | 用途 |
|---|---|
| `Z2WholeBodyClient()` | 创建 wholebody 客户端 |
| `Z2MotorClient(role="leg")` | 创建 leg/body/arm/wholebody 任意控制组客户端 |
| `client.set_motor(gid, mode, pos, vel, tau, kp, kd)` | 写单个电机到外部命令缓冲区 |
| `client.setCommand()` | 更新一次命令快照，后台持续发布 |
| `client.getStates()` | 返回一次后台缓存状态 |
| `wait_for_current_positions(client, timeout=5.0)` | 等待并校验状态，返回 `{gid: pos}` |
| `build_command_maps(WHOLEBODY_TARGET_STANCE)` | 把大数组拆成 positions/kps/kds |
| `command_all_motors(client, positions, kps, kds)` | 兼容辅助函数，内部调用 `set_motor()` + `setCommand()` |
| `interpolate_positions(gids, start, target, alpha)` | 线性插值位置 |
| `print_positions(positions)` | 打印 `{gid: pos}` |
| `print_states(states)` | 打印完整状态表 |
