# by wcy
# 2026/4/1

# ============================================================================
# 本脚本基于 z1_three.py 的 DDS 客户端逻辑，保留相同的：
# 1. DomainParticipant / Topic / DataWriter / DataReader 初始化方式
# 2. 后台线程 500Hz 持续发布命令、读取状态的通信结构
# 3. leg / arm / body 三类控制接口的数据组织方式
#
# 本脚本把主程序改成“单电机模式切换测试”，用于通过 DDS 单独验证
# EYOU / YKS / TI5 三类电机在统一命令接口下的速度/电流控制链路。
# ============================================================================

import time
from copy import deepcopy
import threading

from cyclonedds.core import DDSException
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
import nubotddsmsg.hr as hrmsg

ARMCMDTOPIC = "/nubot/z1/armmotorcmds"
LEGCMDTOPIC = "/nubot/z1/legmotorcmds"
ARMSTATETOPIC = "/nubot/z1/armmotorstates"
LEGSTATETOPIC = "/nubot/z1/legmotorstates"
BODYCMDTOPIC = "/nubot/z1/bodymotorcmds"
BODYSTATETOPIC = "/nubot/z1/bodymotorstates"


class Z1RemoteClient(threading.Thread):
    def __init__(self, cmdtopic, statetopic, role):
        super(Z1RemoteClient, self).__init__()
        self.cmdtopic = cmdtopic
        self.statetopic = statetopic

        if role not in ["arm", "leg", "body"]:
            raise ValueError("role must be 'arm', 'leg' or 'body'")

        motornum = {"arm": 12, "leg": 12, "body": 6}
        levels = {"leg": 0, "arm": 1, "body": 2}
        self.role = role
        self.daemon = True

        self._motorCmds = hrmsg.motorcmds(
            level=levels[role],
            cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])],
        )
        self.motorCmds = hrmsg.motorcmds(
            level=levels[role],
            cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])],
        )
        self._motorStates = hrmsg.motorstates(
            level=levels[role],
            states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])],
        )

        participant = DomainParticipant(domain_id=0)

        cmdtopic_obj = Topic(participant, self.cmdtopic, hrmsg.motorcmds)
        cmdqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5),
        )
        publisher = Publisher(participant)
        self.writer = DataWriter(publisher, cmdtopic_obj, qos=cmdqos)

        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5),
        )
        statetopic_obj = Topic(participant, self.statetopic, hrmsg.motorstates, qos=stateqos)
        self.reader = DataReader(participant, statetopic_obj)

        self.running = True
        self.reader_valid = False
        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()

        self.start()

    def run(self):
        while self.running:
            msgs = []
            try:
                msgs = self.reader.take()
                self.reader_valid = bool(msgs)
            except DDSException as exc:
                print("[Reader] catch DDSException msg:", exc.msg)
            except TimeoutError:
                print("[Reader] take sample timeout")
            except Exception:
                print("[Reader] take sample error")

            if msgs:
                with self._lockstate:
                    self._motorStates = msgs[-1]

            with self._lockcmd:
                try:
                    self.writer.write(self._motorCmds)
                except DDSException as exc:
                    print("[Writer] catch DDSException error. msg:", exc.msg)
                except Exception as exc:
                    print("[Writer] write sample error. msg:", exc)

            time.sleep(0.002)

    def stop(self):
        self.running = False
        self.join(timeout=1.0)

    def setCommand(self):
        with self._lockcmd:
            self._motorCmds = deepcopy(self.motorCmds)

    def getStates(self):
        with self._lockstate:
            return deepcopy(self._motorStates)

    def arm_ti5_squat_control(self, arm_index, value, mode):
        arm_index = arm_index - 12
        if not (0 <= arm_index < len(self.motorCmds.cmds)):
            raise IndexError("Invalid motor index")
        cmd = self.motorCmds.cmds[arm_index]
        cmd.mode = mode
        if mode == 1:
            cmd.pos = value
        elif mode == 2:
            cmd.tau = value
        elif mode == 3:
            cmd.vel = value

    def arm_yks_squat_control(self, arm_index, mode, pos, vel, tau, kp, kd):
        if not (12 <= arm_index < 24):
            raise IndexError("Invalid motor index")
        arm_index = arm_index - 12
        cmd = self.motorCmds.cmds[arm_index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    def body_yks_squat_control(self, body_index, mode, pos, vel, tau, kp, kd):
        if not (24 <= body_index < 30):
            raise IndexError("Invalid motor index")
        body_index = body_index - 24
        cmd = self.motorCmds.cmds[body_index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    def leg_squat_control(self, leg_index, mode, pos, vel, tau, kp, kd):
        if not (0 <= leg_index < 12):
            raise IndexError("Invalid motor index")
        cmd = self.motorCmds.cmds[leg_index]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd


TEST_ROLE = "leg"
TEST_MOTOR_FAMILY = "eyou"
TEST_GLOBAL_INDEX = 1
TEST_MODE = "position"
TEST_SPEED = 6.28
TEST_CURRENT = 200.0
TEST_POSITION = 0.0
TEST_POSITION_OFFSET = 3.14
TEST_POSITION_USE_OFFSET = True
TEST_POSITION_PROFILE_SPEED = 1.0
TEST_SEND_INTERVAL = 0.02
TEST_PRINT_INTERVAL = 0.5
TEST_STOP_HOLD = 0.3

MODE_TO_CODE = {
    "position": 1,
    "speed": 3,
    "current": 2,
}


def create_client(role):
    if role == "leg":
        return Z1RemoteClient(LEGCMDTOPIC, LEGSTATETOPIC, "leg")
    if role == "arm":
        return Z1RemoteClient(ARMCMDTOPIC, ARMSTATETOPIC, "arm")
    if role == "body":
        return Z1RemoteClient(BODYCMDTOPIC, BODYSTATETOPIC, "body")
    raise ValueError("Unsupported role")


def clear_all_commands(client):
    for cmd in client.motorCmds.cmds:
        cmd.mode = 0
        cmd.pos = 0.0
        cmd.vel = 0.0
        cmd.tau = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0


def get_local_index(role, global_index):
    if role == "leg":
        return global_index
    if role == "arm":
        return global_index - 12
    if role == "body":
        return global_index - 24
    raise ValueError("Unsupported role")


def build_test_command(test_mode, target_value):
    if test_mode == "position":
        return {
            "mode": MODE_TO_CODE[test_mode],
            "pos": target_value,
            "vel": TEST_POSITION_PROFILE_SPEED,
            "tau": 0.0,
            "kp": 0.0,
            "kd": 0.0,
        }

    if test_mode == "speed":
        return {
            "mode": MODE_TO_CODE[test_mode],
            "pos": 0.0,
            "vel": target_value,
            "tau": 0.0,
            "kp": 0.0,
            "kd": 0.0,
        }

    if test_mode == "current":
        return {
            "mode": MODE_TO_CODE[test_mode],
            "pos": 0.0,
            "vel": 0.0,
            "tau": target_value,
            "kp": 0.0,
            "kd": 0.0,
        }

    raise ValueError("Unsupported test mode, use 'position', 'speed' or 'current'")


def apply_test_command(client, role, motor_family, global_index, test_mode, target_value):
    clear_all_commands(client)
    command = build_test_command(test_mode, target_value)
    mode = command["mode"]
    pos = command["pos"]
    vel = command["vel"]
    tau = command["tau"]
    kp = command["kp"]
    kd = command["kd"]

    if role == "leg":
        client.leg_squat_control(global_index, mode, pos, vel, tau, kp, kd)
        return get_local_index(role, global_index), command

    if role == "body":
        client.body_yks_squat_control(global_index, mode, pos, vel, tau, kp, kd)
        return get_local_index(role, global_index), command

    if role == "arm":
        if motor_family == "ti5" and test_mode != "speed":
            raise ValueError("TI5 test script currently only supports speed mode")

        if motor_family == "ti5":
            client.arm_ti5_squat_control(global_index, target_value, mode)
        else:
            client.arm_yks_squat_control(global_index, mode, pos, vel, tau, kp, kd)
        return get_local_index(role, global_index), command

    raise ValueError("Unsupported role")


def get_test_target_value(test_mode):
    if test_mode == "position":
        return TEST_POSITION
    if test_mode == "speed":
        return TEST_SPEED
    if test_mode == "current":
        return TEST_CURRENT
    raise ValueError("Unsupported test mode, use 'position', 'speed' or 'current'")


def wait_for_valid_state(client, timeout=1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if client.reader_valid:
            return client.getStates()
        time.sleep(0.01)
    return client.getStates()


def resolve_position_target(client, role, global_index):
    states = wait_for_valid_state(client)
    local_index = get_local_index(role, global_index)
    if 0 <= local_index < len(states.states):
        current_position = states.states[local_index].pos
    else:
        current_position = 0.0

    if TEST_POSITION_USE_OFFSET:
        return current_position + TEST_POSITION_OFFSET, current_position

    return TEST_POSITION, current_position


def send_zero_target_for_stop(client, role, motor_family, global_index, test_mode):
    if test_mode == "position":
        states = client.getStates()
        local_index = get_local_index(role, global_index)
        if 0 <= local_index < len(states.states):
            stop_target = states.states[local_index].pos
        else:
            stop_target = 0.0
    else:
        stop_target = 0.0

    stop_end_time = time.time() + TEST_STOP_HOLD
    while time.time() < stop_end_time:
        apply_test_command(
            client,
            role,
            motor_family,
            global_index,
            test_mode,
            stop_target,
        )
        client.setCommand()
        time.sleep(TEST_SEND_INTERVAL)


def print_motor_state(states, local_index, role, global_index):
    if local_index < 0 or local_index >= len(states.states):
        print(f"[State] role={role} global_index={global_index} local_index={local_index} out of range")
        return

    state = states.states[local_index]
    print(
        f"[State] role={role} global_index={global_index} local_index={local_index} "
        f"mode={state.mode} pos={state.pos:.4f} vel={state.vel:.4f} tau={state.tau:.4f} err={state.error}"
    )


def print_test_command(test_mode, command):
    print(
        f"[Command] mode={test_mode} code={command['mode']} "
        f"pos={command['pos']:.4f} vel={command['vel']:.4f} tau={command['tau']:.4f}"
    )


if __name__ == "__main__":
    client = create_client(TEST_ROLE)
    if TEST_MODE == "position":
        test_target, current_position = resolve_position_target(
            client,
            TEST_ROLE,
            TEST_GLOBAL_INDEX,
        )
    else:
        test_target = get_test_target_value(TEST_MODE)
        current_position = None

    local_index, command = apply_test_command(
        client,
        TEST_ROLE,
        TEST_MOTOR_FAMILY,
        TEST_GLOBAL_INDEX,
        TEST_MODE,
        test_target,
    )

    print(
        f"[DDS Test] role={TEST_ROLE}, family={TEST_MOTOR_FAMILY}, "
        f"global_index={TEST_GLOBAL_INDEX}, mode={TEST_MODE}, target={test_target}"
    )
    if TEST_MODE == "position":
        print(
            f"[DDS Test] profile_speed={TEST_POSITION_PROFILE_SPEED}, "
            f"current_pos={current_position}, use_offset={TEST_POSITION_USE_OFFSET}, "
            f"offset={TEST_POSITION_OFFSET}"
        )

    last_print = 0.0
    try:
        while True:
            local_index, command = apply_test_command(
                client,
                TEST_ROLE,
                TEST_MOTOR_FAMILY,
                TEST_GLOBAL_INDEX,
                TEST_MODE,
                test_target,
            )
            client.setCommand()

            now = time.time()
            if now - last_print >= TEST_PRINT_INTERVAL:
                print_test_command(TEST_MODE, command)
                states = client.getStates()
                print_motor_state(states, local_index, TEST_ROLE, TEST_GLOBAL_INDEX)
                last_print = now

            time.sleep(TEST_SEND_INTERVAL)
    except KeyboardInterrupt:
        print("\n[DDS Test] stop requested")
    finally:
        send_zero_target_for_stop(
            client,
            TEST_ROLE,
            TEST_MOTOR_FAMILY,
            TEST_GLOBAL_INDEX,
            TEST_MODE,
        )
        client.stop()