# by mrtang
# 2025/4/12

import math
import time
from pyexpat.errors import messages

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.qos import Qos, Policy
from cyclonedds.core import DDSException, Listener

import nubotddsmsg.hr as hrmsg
from copy import deepcopy

import threading

from cyclonedds.idl import IdlStruct, IdlUnion, IdlBitmask, IdlEnum, types


ARMCMDTOPIC = "/nubot/z1/armmotorcmds"
LEGCMDTOPIC = "/nubot/z1/legmotorcmds"

ARMSTATETOPIC = "/nubot/z1/armmotorstates"
LEGSTATETOPIC = "/nubot/z1/legmotorstates"

BODYCMDTOPIC = "/nubot/z1/bodymotorcmds"
BODYSTATETOPIC = "/nubot/z1/bodymotorstates"

LEG_GIDS = [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13]
BODY_GIDS = [16, 17, 18]
ARM_GIDS = [24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38]

class Z1RemoteClient(threading.Thread):
    def __init__(self, cmdtopic, statetopic, role):
        '''
        :param cmdtopic:   发布命令的topic
        :param statetopic: 订阅状态的topic
        :param role:       leg-下肢， arm-上肢 ，body-躯干
        '''

        super(Z1RemoteClient, self).__init__()

        self.cmdtopic = cmdtopic
        self.statetopic = statetopic

        if role not in ['arm', 'leg','body']:
            raise ValueError("role must be 'arm', 'leg', or 'body'")

        motornum = {'arm': 14, 'leg': 12, 'body': 3}
        levels = {'leg': 0, 'arm': 1, 'body': 2}

        self.daemon = True

        # 初始化消息
        self._motorCmds = hrmsg.motorcmds(level=levels[role],
                                          cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        self.motorCmds = hrmsg.motorcmds(level=levels[role],
                                         cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in range(motornum[role])])

        self._motorStates = hrmsg.motorstates(level=levels[role],
                                              states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in
                                                      range(motornum[role])])

        participant = DomainParticipant(domain_id=0)

        ###########################################################################
        ### 发布
        cmdtopic = Topic(participant, self.cmdtopic, hrmsg.motorcmds)
        cmdqos = Qos(
            Policy.Reliability.BestEffort,  # 或 Policy.Reliability.Reliable
            Policy.Durability.Volatile,  # 或 Policy.Durability.TransientLocal
            Policy.History.KeepLast(5),  # 保留最后5条消息
        )
        publisher = Publisher(participant)
        self.writer = DataWriter(publisher, cmdtopic, qos=cmdqos)

        ###########################################################################
        ### 订阅
        stateqos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.History.KeepLast(5)
        )
        statetopic = Topic(participant, self.statetopic, hrmsg.motorstates, qos=stateqos)
        self.reader = DataReader(participant, statetopic)

        self.running = True
        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()
        self.reader_valid = False
        self._has_state = False
        self._last_state_time = 0.0

        self.start()
        # time.sleep(0.1)



    def run(self):

        while self.running:
            msgs = []
            ## 处理订阅
            try:
                msgs = self.reader.take()
                if not msgs:
                    self.reader_valid = False
                else:
                    self.reader_valid = True
            except DDSException as e:
                print("[Reader] catch DDSException msg:", e.msg)
            except TimeoutError as e:
                print("[Reader] take sample timeout")
            except:
                print("[Reader] take sample error")
            if len(msgs) > 0:
                self._lockstate.acquire()
                self._motorStates = msgs[-1]
                self._has_state = True
                self._last_state_time = time.monotonic()
                self._lockstate.release()

            ## 处理发布
            self._lockcmd.acquire()
            try:
                self.writer.write(self._motorCmds)
            except DDSException as e:
                print("[Writer] catch DDSException error. msg:", e.msg)
            except Exception as e:
                print("[Writer] write sample error. msg:", e.args())
            except:
                print("[Writer] write sample error.")

            self._lockcmd.release()

            time.sleep(0.002)  # 500Hz

    def stop(self):
        self.running = False

    def setCommand(self):
        self._lockcmd.acquire()
        self._motorCmds = deepcopy(self.motorCmds)
        self._lockcmd.release()

    def getStates(self):  # 返回值 nubotddsmsg.hr.motorstates
        self._lockstate.acquire()
        states = deepcopy(self._motorStates)
        self._lockstate.release()
        return states

    def hasState(self):
        self._lockstate.acquire()
        has_state = self._has_state
        self._lockstate.release()
        return has_state


    def arm_ti5_squat_control(self, arm_index, value, mode):
        """
            :param arm_index: 关节全局ID
            :param value: 目标值
            :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
        """
        if arm_index not in ARM_GIDS:
            raise IndexError("Invalid motor index")
        local = ARM_GIDS.index(arm_index)

        cmd = self.motorCmds.cmds[local]
        cmd.mode = mode

        if mode == 1:
            cmd.pos = value
        elif mode == 2:
            cmd.tau = value
        elif mode == 3:
            cmd.vel = value

    def arm_yks_squat_control(self, arm_index, mode, pos, vel, tau, kp, kd):
        """
        :param arm_index: 关节全局ID
        :param value: 目标值
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
        """
        if arm_index not in ARM_GIDS:
            raise IndexError("Invalid motor index")
        local = ARM_GIDS.index(arm_index)

        cmd = self.motorCmds.cmds[local]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd
    def body_yks_squat_control(self, body_index, mode, pos, vel, tau, kp, kd):
        """
        :param body_index: 关节全局ID
        :param value: 目标值
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
        """
        if body_index not in BODY_GIDS:
            raise IndexError("Invalid motor index")
        local = BODY_GIDS.index(body_index)

        cmd = self.motorCmds.cmds[local]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    def leg_squat_control(self, leg_index, mode, pos, vel, tau, kp, kd):
        """
        :param leg_index: 关节全局ID
        :param value: 目标值
        :param mode: 控制模式 (1: position, 2: torque, 3: velocity)
        """
        if leg_index not in LEG_GIDS:
            raise IndexError("Invalid motor index")
        local = LEG_GIDS.index(leg_index)

        cmd = self.motorCmds.cmds[local]
        cmd.mode = mode
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    def read_arm_ti5_control(self, arm_index, mode):
        """
        :param arm_index: 关节索引（全局ID）
        :param mode: 读取模式 (0: 力位混合 1: position, 2: torque, 3: velocity)
        :return: 对应模式下的状态值
        """
        if arm_index not in ARM_GIDS:
            raise IndexError("Invalid motor index")
        local = ARM_GIDS.index(arm_index)

        state = self._motorStates.states[local]

        if mode == 1:
            return state.pos
        elif mode == 2:
            return state.tau
        elif mode == 3:
            return state.vel
        else:
            return 0


def wait_for_current_positions(client, gids, role, timeout=5.0):
    deadline = time.monotonic() + timeout
    last_error = "no state sample received"

    while time.monotonic() < deadline:
        states = client.getStates()
        if client.hasState() and len(states.states) >= len(gids):
            positions = {}
            valid = True
            for local, gid in enumerate(gids):
                state = states.states[local]
                state_gid = int(state.index)
                pos = float(state.pos)

                if state_gid != gid:
                    last_error = (
                        f"{role} state index mismatch at local {local}: "
                        f"got {state_gid}, expected {gid}"
                    )
                    valid = False
                    break
                if not math.isfinite(pos):
                    last_error = f"{role} gid {gid} position is not finite: {pos}"
                    valid = False
                    break
                positions[gid] = pos

            if valid:
                return positions
        elif len(states.states) < len(gids):
            last_error = f"{role} state length {len(states.states)} < expected {len(gids)}"

        time.sleep(0.01)

    raise TimeoutError(f"Timed out waiting for {role} current positions: {last_error}")


def print_positions(role, positions):
    print(f"{role} current positions:")
    for gid in sorted(positions):
        print(f"  gid {gid:2d}: {positions[gid]: .4f}")


def gain_at(gains, gid, local):
    if isinstance(gains, dict):
        return gains[gid]
    return gains[local]


def command_position(client, role, gid, pos, kp, kd, mode=0, vel=0.0, tau=0.0):
    if role == "leg":
        client.leg_squat_control(gid, mode, pos, vel, tau, kp, kd)
    elif role == "body":
        client.body_yks_squat_control(gid, mode, pos, vel, tau, kp, kd)
    elif role == "arm":
        client.arm_yks_squat_control(gid, mode, pos, vel, tau, kp, kd)
    else:
        raise ValueError(f"Unsupported role: {role}")


def command_positions(client, role, gids, positions, kps, kds):
    for local, gid in enumerate(gids):
        command_position(
            client,
            role,
            gid,
            positions[gid],
            gain_at(kps, gid, local),
            gain_at(kds, gid, local),
        )


def publish_all(*clients):
    for client in clients:
        client.setCommand()


def ramp_to_targets(groups, duration=10.0, period=0.02):
    steps = max(1, int(duration / period))
    for step in range(steps + 1):
        alpha = step / steps
        for group in groups:
            positions = {}
            for gid in group["gids"]:
                start = group["start"][gid]
                target = group["target"][gid]
                positions[gid] = start + (target - start) * alpha

            command_positions(
                group["client"],
                group["role"],
                group["gids"],
                positions,
                group["kp"],
                group["kd"],
            )

        publish_all(*(group["client"] for group in groups))
        time.sleep(period)



if __name__ == '__main__':
    z1_arm = Z1RemoteClient(ARMCMDTOPIC, ARMSTATETOPIC, 'arm')


    z1_leg = Z1RemoteClient(LEGCMDTOPIC, LEGSTATETOPIC, 'leg')

    z1_body = Z1RemoteClient(BODYCMDTOPIC, BODYSTATETOPIC, 'body')

    joint_angles = [0.0, 0, 0, 0.0, 0.0, 0,
                    0.0, 0, 0, 0.0, 0.0, 0]
    ksp_stance = [500, 500, 500, 500, 500, 500,
                  500, 500, 500, 500, 500, 500]
    ksd_stance = [50, 50, 5, 50, 20, 20,
                  50, 50, 5, 50, 20, 20]
    ramp_duration = 10.0
    ramp_period = 0.02

    leg_targets = dict(zip(LEG_GIDS, joint_angles))
    body_targets = {gid: 0.0 for gid in BODY_GIDS}
    arm_targets = {gid: 0.0 for gid in ARM_GIDS}
    body_kp = {gid: 100 for gid in BODY_GIDS}
    body_kd = {gid: 10 for gid in BODY_GIDS}
    arm_kp = {gid: 100 for gid in ARM_GIDS}
    arm_kd = {gid: 5 for gid in ARM_GIDS}

    try:
        print("Waiting for current motor positions...")
        leg_start = wait_for_current_positions(z1_leg, LEG_GIDS, "leg")
        body_start = wait_for_current_positions(z1_body, BODY_GIDS, "body")
        arm_start = wait_for_current_positions(z1_arm, ARM_GIDS, "arm")

        print_positions("leg", leg_start)
        print_positions("body", body_start)
        print_positions("arm", arm_start)

        # First hold the measured positions, then move slowly to the targets.
        command_positions(z1_leg, "leg", LEG_GIDS, leg_start, ksp_stance, ksd_stance)
        command_positions(z1_body, "body", BODY_GIDS, body_start, body_kp, body_kd)
        command_positions(z1_arm, "arm", ARM_GIDS, arm_start, arm_kp, arm_kd)
        publish_all(z1_leg, z1_body, z1_arm)
        time.sleep(0.5)

        print(f"Moving to target positions in {ramp_duration:.1f}s...")
        groups = [
            {
                "client": z1_leg,
                "role": "leg",
                "gids": LEG_GIDS,
                "start": leg_start,
                "target": leg_targets,
                "kp": ksp_stance,
                "kd": ksd_stance,
            },
            {
                "client": z1_body,
                "role": "body",
                "gids": BODY_GIDS,
                "start": body_start,
                "target": body_targets,
                "kp": body_kp,
                "kd": body_kd,
            },
            {
                "client": z1_arm,
                "role": "arm",
                "gids": ARM_GIDS,
                "start": arm_start,
                "target": arm_targets,
                "kp": arm_kp,
                "kd": arm_kd,
            },
        ]
        ramp_to_targets(groups, duration=ramp_duration, period=ramp_period)
        print("Target positions reached. Holding command.")

        while True:
            publish_all(z1_leg, z1_body, z1_arm)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        z1_leg.stop()
        z1_body.stop()
        z1_arm.stop()
