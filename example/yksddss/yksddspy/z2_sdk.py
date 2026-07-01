#!/usr/bin/env python3
"""
Z2 DDS Python SDK.

The SDK keeps all Z1/Z2 motor DDS topics in one place and provides a generic
client for arm, leg, body, and wholebody command/state pairs.

The client uses a background thread, like the older Z1 examples:
  - the background thread continuously takes states and writes commands
  - user calls set_motor() to edit motorCmds
  - user calls setCommand() to update the command snapshot once
  - user calls getStates() to read one cached state snapshot
"""

import math
import threading
import time
from copy import deepcopy
from dataclasses import dataclass

from cyclonedds.core import DDSException
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
import nubotddsmsg.hr as hrmsg


ARMCMDTOPIC = "/nubot/z1/armmotorcmds"
LEGCMDTOPIC = "/nubot/z1/legmotorcmds"
BODYCMDTOPIC = "/nubot/z1/bodymotorcmds"
WHOLEBODYCMDTOPIC = "/nubot/z1/wholebodymotorcmds"

ARMSTATETOPIC = "/nubot/z1/armmotorstates"
LEGSTATETOPIC = "/nubot/z1/legmotorstates"
BODYSTATETOPIC = "/nubot/z1/bodymotorstates"
WHOLEBODYSTATETOPIC = "/nubot/z1/wholebodymotorstates"

LEG_GIDS = [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13]
BODY_GIDS = [16, 17, 18]
ARM_GIDS = [24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38]
ALL_GIDS = LEG_GIDS + BODY_GIDS + ARM_GIDS
WHOLEBODY_GIDS = ALL_GIDS
ACTIVE_MOTOR_NUM = len(WHOLEBODY_GIDS)

GID_TO_JOINT = {
    0: "LeftHipPitch", 1: "LeftHipRoll", 2: "LeftHipYaw", 3: "LeftKnee",
    4: "LeftAnklePitch/B", 5: "LeftAnkleRoll/A",
    8: "RightHipPitch", 9: "RightHipRoll", 10: "RightHipYaw", 11: "RightKnee",
    12: "RightAnklePitch/B", 13: "RightAnkleRoll/A",
    16: "WaistRoll/A", 17: "WaistPitch/B", 18: "WaistYaw",
    24: "LeftShoulderPitch", 25: "LeftShoulderRoll", 26: "LeftShoulderYaw",
    27: "LeftElbow", 28: "LeftForearmRoll", 29: "LeftWristYaw",
    30: "LeftWristPitch",
    32: "RightShoulderPitch", 33: "RightShoulderRoll", 34: "RightShoulderYaw",
    35: "RightElbow", 36: "RightForearmRoll", 37: "RightWristYaw",
    38: "RightWristPitch",
}

# Each row is: (global_id, target_pos, kp, kd). Keep this sorted by ALL_GIDS.
WHOLEBODY_TARGET_STANCE = [
    # (0, 0.0, 500, 50),
    # (1, 0.0, 500, 50),
    # (2, 0.0, 500, 5),
    # (3, 0.0, 500, 50),
    # (4, 0.0, 500, 20),
    # (5, 0.0, 500, 20),
    # (8, 0.0, 500, 50),
    # (9, 0.0, 500, 50),
    # (10, 0.0, 500, 5),
    # (11, 0.0, 500, 50),
    # (12, 0.0, 500, 20),
    # (13, 0.0, 500, 20),
    # (16, 0.0, 100, 10),
    # (17, 0.0, 100, 10),
    # (18, 0.0, 100, 10),
    # (24, 0.0, 100, 10),
    # (25, 0.0, 100, 10),
    # (26, 0.0, 100, 10),
    # (27, 0.0, 100, 10),
    # (28, 0.0, 100, 10),
    # (29, 0.0, 100, 10),
    # (30, 0.0, 100, 10),
    # (32, 0.0, 100, 10),
    # (33, 0.0, 100, 10),
    # (34, 0.0, 100, 10),
    # (35, 0.0, 100, 10),
    # (36, 0.0, 100, 10),
    # (37, 0.0, 100, 10),
    # (38, 0.0, 100, 10),
#到达对应角度
    (0, 0.63, 500, 50),
    (1, 0.73, 500, 50),
    (2, 0.2, 500, 5),
    (3, 0.64, 500, 50),
    (4, 0.22, 500, 20),
    (5, 0.0, 500, 20),
    (8, -0.55, 500, 50),
    (9, 0.16, 500, 50),
    (10, -0.1, 500, 5),
    (11, 0.44, 500, 50),
    (12, -0.2, 500, 20),
    (13, 0.21, 500, 20),
    (16, 0.0, 100, 10),
    (17, 0.0, 100, 10),
    (18, 0.2, 100, 10),
    (24, -0.83, 100, 10),
    (25, 0.7, 100, 10),
    (26, -0.7, 100, 10),
    (27, -0.2, 100, 10),
    (28, 0.0, 100, 10),
    (29, 0.0, 100, 10),
    (30, 0.0, 100, 10),
    (32, 0.9, 100, 10),
    (33, -1.0, 100, 10),
    (34, 0.55, 100, 10),
    (35, 0.0, 100, 10),
    (36, 0.0, 100, 10),
    (37, 0.0, 100, 10),
    (38, 0.0, 100, 10),
]


@dataclass(frozen=True)
class MotorGroupConfig:
    role: str
    level: int
    gids: list
    cmdtopic: str
    statetopic: str


MOTOR_GROUPS = {
    "leg": MotorGroupConfig("leg", 0, LEG_GIDS, LEGCMDTOPIC, LEGSTATETOPIC),
    "arm": MotorGroupConfig("arm", 1, ARM_GIDS, ARMCMDTOPIC, ARMSTATETOPIC),
    "body": MotorGroupConfig("body", 2, BODY_GIDS, BODYCMDTOPIC, BODYSTATETOPIC),
    "wholebody": MotorGroupConfig(
        "wholebody", 3, WHOLEBODY_GIDS, WHOLEBODYCMDTOPIC, WHOLEBODYSTATETOPIC
    ),
}


def _default_qos():
    return Qos(
        Policy.Reliability.BestEffort,
        Policy.Durability.Volatile,
        Policy.History.KeepLast(5),
    )


def _value_at(values, gid, local):
    if isinstance(values, dict):
        return values[gid]
    return values[local]


class Z2MotorClient(threading.Thread):
    """Background DDS command/state client for one motor group."""

    def __init__(self, role="wholebody", cmdtopic=None, statetopic=None, domain_id=0,
                 publish_period=0.002):
        if role not in MOTOR_GROUPS:
            raise ValueError(f"role must be one of {sorted(MOTOR_GROUPS)}")

        super().__init__(daemon=True)

        config = MOTOR_GROUPS[role]
        self.role = config.role
        self.level = config.level
        self.gids = list(config.gids)
        self.gid_to_local = {gid: i for i, gid in enumerate(self.gids)}
        self.cmdtopic = cmdtopic or config.cmdtopic
        self.statetopic = statetopic or config.statetopic
        self.publish_period = publish_period

        self._motorCmds = self._empty_cmds()
        self.motorCmds = self._empty_cmds()
        self._motorStates = self._empty_states()

        self.running = True
        self.reader_valid = False
        self._has_state = False
        self._last_state_time = 0.0
        self._lockcmd = threading.RLock()
        self._lockstate = threading.RLock()

        self.participant = DomainParticipant(domain_id=domain_id)
        qos = _default_qos()

        cmd_topic = Topic(self.participant, self.cmdtopic, hrmsg.motorcmds)
        publisher = Publisher(self.participant)
        self.writer = DataWriter(publisher, cmd_topic, qos=qos)

        state_topic = Topic(self.participant, self.statetopic, hrmsg.motorstates, qos=qos)
        self.reader = DataReader(self.participant, state_topic)

        self.start()

    def _empty_cmds(self):
        return hrmsg.motorcmds(
            level=self.level,
            cmds=[hrmsg.motorcmd(0, 0, 0, 0, 0, 0, 0) for _ in self.gids],
        )

    def _empty_states(self):
        return hrmsg.motorstates(
            level=self.level,
            states=[hrmsg.motorstate(0, 0, 0, 0, 0, 0, 0, 0, 0, 0) for _ in self.gids],
        )

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
            except Exception as exc:
                print("[Reader] take sample error:", exc)

            if msgs:
                with self._lockstate:
                    self._motorStates = deepcopy(msgs[-1])
                    self._has_state = True
                    self._last_state_time = time.monotonic()

            with self._lockcmd:
                try:
                    self.writer.write(self._motorCmds)
                except DDSException as exc:
                    print("[Writer] catch DDSException error. msg:", exc.msg)
                except Exception as exc:
                    print("[Writer] write sample error:", exc)

            time.sleep(self.publish_period)

    def make_command(self, positions, kps, kds, mode=0, vel=0.0, tau=0.0):
        cmds = self._empty_cmds()
        for local, gid in enumerate(self.gids):
            cmd = cmds.cmds[local]
            cmd.mode = mode
            cmd.index = gid
            cmd.pos = float(_value_at(positions, gid, local))
            cmd.vel = float(vel)
            cmd.tau = float(tau)
            cmd.kp = float(_value_at(kps, gid, local))
            cmd.kd = float(_value_at(kds, gid, local))
        return cmds

    def set_motor(self, gid, mode, pos, vel=0.0, tau=0.0, kp=0.0, kd=0.0):
        if gid not in self.gid_to_local:
            raise IndexError(f"Invalid motor gid: {gid}, valid gids: {self.gids}")

        local = self.gid_to_local[gid]
        cmd = self.motorCmds.cmds[local]
        cmd.mode = mode
        cmd.index = gid
        cmd.pos = pos
        cmd.vel = vel
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    def set_command(self, cmds):
        with self._lockcmd:
            self.motorCmds = deepcopy(cmds)
            self._motorCmds = deepcopy(cmds)
        return cmds

    def setCommand(self):
        with self._lockcmd:
            self._motorCmds = deepcopy(self.motorCmds)
        return self._motorCmds

    def write_command(self, positions, kps, kds, mode=0, vel=0.0, tau=0.0):
        cmds = self.make_command(positions, kps, kds, mode=mode, vel=vel, tau=tau)
        return self.set_command(cmds)

    def get_states(self):
        with self._lockstate:
            return deepcopy(self._motorStates)

    def getStates(self):
        return self.get_states()

    def has_state(self):
        with self._lockstate:
            return self._has_state

    def hasState(self):
        return self.has_state()

    def read_state_once(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        last_error = "no state sample received"

        while time.monotonic() < deadline:
            with self._lockstate:
                has_state = self._has_state
                states = deepcopy(self._motorStates)

            if has_state:
                if len(states.states) >= len(self.gids):
                    return states
                last_error = (
                    f"{self.role} state length {len(states.states)} "
                    f"< expected {len(self.gids)}"
                )

            time.sleep(0.01)

        raise TimeoutError(f"Timed out waiting for {self.role} states: {last_error}")

    def read_positions(self, timeout=5.0):
        states = self.read_state_once(timeout=timeout)
        positions = {}

        for local, gid in enumerate(self.gids):
            state = states.states[local]
            state_gid = int(state.index)
            pos = float(state.pos)

            if state_gid != gid:
                raise ValueError(
                    f"{self.role} state index mismatch at local {local}: "
                    f"got {state_gid}, expected {gid}"
                )
            if not math.isfinite(pos):
                raise ValueError(f"{self.role} gid {gid} position is not finite: {pos}")

            positions[gid] = pos

        return positions

    def read_motor_state(self, gid, attr="pos", timeout=5.0):
        if gid not in self.gid_to_local:
            raise IndexError(f"Invalid motor gid: {gid}, valid gids: {self.gids}")
        states = self.read_state_once(timeout=timeout)
        state = states.states[self.gid_to_local[gid]]
        return getattr(state, attr)

    def stop(self, join=True, timeout=1.0):
        self.running = False
        if join and self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=timeout)


class Z2WholeBodyClient(Z2MotorClient):
    def __init__(self, cmdtopic=WHOLEBODYCMDTOPIC, statetopic=WHOLEBODYSTATETOPIC,
                 domain_id=0, publish_period=0.002):
        super().__init__(
            role="wholebody",
            cmdtopic=cmdtopic,
            statetopic=statetopic,
            domain_id=domain_id,
            publish_period=publish_period,
        )


def build_command_maps(commands, gids=None):
    expected_gids = list(gids or WHOLEBODY_GIDS)
    command_gids = [int(row[0]) for row in commands]
    if command_gids != expected_gids:
        raise ValueError(
            "command rows must match the target gids and be sorted by global_id: "
            f"got {command_gids}, expected {expected_gids}"
        )

    target_positions = {}
    kps = {}
    kds = {}
    for gid, target_pos, kp, kd in commands:
        target_positions[int(gid)] = float(target_pos)
        kps[int(gid)] = float(kp)
        kds[int(gid)] = float(kd)

    return target_positions, kps, kds


def command_all_motors(client, positions, kps, kds, mode=0, vel=0.0, tau=0.0):
    for local, gid in enumerate(client.gids):
        client.set_motor(
            gid,
            mode,
            _value_at(positions, gid, local),
            vel=vel,
            tau=tau,
            kp=_value_at(kps, gid, local),
            kd=_value_at(kds, gid, local),
        )
    return client.setCommand()


def publish_all(*clients):
    for client in clients:
        client.setCommand()


def wait_for_current_positions(client, timeout=5.0):
    deadline = time.monotonic() + timeout
    last_error = "no state sample received"

    while time.monotonic() < deadline:
        states = client.getStates()

        if client.hasState() and len(states.states) >= len(client.gids):
            positions = {}
            valid = True

            for local, gid in enumerate(client.gids):
                state = states.states[local]
                state_gid = int(state.index)
                pos = float(state.pos)

                if state_gid != gid:
                    last_error = (
                        f"{client.role} state index mismatch at local {local}: "
                        f"got {state_gid}, expected {gid}"
                    )
                    valid = False
                    break

                if not math.isfinite(pos):
                    last_error = f"{client.role} gid {gid} position is not finite: {pos}"
                    valid = False
                    break

                positions[gid] = pos

            if valid:
                return positions

        elif len(states.states) < len(client.gids):
            last_error = f"{client.role} state length {len(states.states)} < expected {len(client.gids)}"

        time.sleep(0.01)

    raise TimeoutError(f"Timed out waiting for {client.role} current positions: {last_error}")


def interpolate_positions(gids, start_positions, target_positions, alpha):
    positions = {}
    for gid in gids:
        start = start_positions[gid]
        target = target_positions[gid]
        positions[gid] = start + (target - start) * alpha
    return positions


def print_positions(positions, title="motor positions"):
    print(f"{title}:")
    for gid in sorted(positions):
        joint_name = GID_TO_JOINT.get(gid, f"Motor{gid}")
        print(f"  gid {gid:2d} ({joint_name:20s}): {positions[gid]: .4f} rad")


def print_state_header():
    print(f"{'idx':>3} {'gid':>3} {'joint':>22s} {'pos(rad)':>9} {'vel(rad/s)':>10} "
          f"{'tau(Nm)':>8} {'cur(A)':>8} {'temp(C)':>7} {'err':>4}")
    print("-" * 85)


def print_states(states):
    print_state_header()
    for index, state in enumerate(states.states):
        gid = int(state.index)
        joint_name = GID_TO_JOINT.get(gid, f"Motor{gid}")
        print(f"{index:3d} {gid:3d} {joint_name:>22s} {state.pos:9.4f} "
              f"{state.vel:10.4f} {state.tau:8.4f} {state.cur:8.4f} "
              f"{state.tem:7.2f} {state.error:4d}")


def main():
    target_positions, kps, kds = build_command_maps(WHOLEBODY_TARGET_STANCE)
    client = Z2WholeBodyClient()

    try:
        print(f"Receiving one cached state sample from {client.statetopic} ...")
        wait_for_current_positions(client, timeout=5.0)
        print_states(client.getStates())

        print(f"Updating one target command for {client.cmdtopic} ...")
        command_all_motors(client, target_positions, kps, kds)
        time.sleep(0.05)
        print("Command snapshot updated; background publisher sent it.")
    except (TimeoutError, ValueError) as exc:
        print(f"ERROR: {exc}")
    finally:
        client.stop()


if __name__ == "__main__":
    main()
