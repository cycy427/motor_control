#!/usr/bin/env python3
"""
Wholebody slow target mover.

This example inherits the wholebody DDS client from z2_sdk.py, then moves all
active motors slowly to WHOLEBODY_TARGET_STANCE and keeps holding the target.
"""

import time

from z2_sdk import (
    ALL_GIDS,
    GID_TO_JOINT,
    WHOLEBODY_TARGET_STANCE,
    Z2WholeBodyClient,
    build_command_maps,
    interpolate_positions,
    print_positions,
    wait_for_current_positions,
)


RAMP_DURATION = 10.0
RAMP_PERIOD = 0.02
HOLD_BEFORE_RAMP = 0.5
HOLD_PERIOD = 0.1


class Z2WholeBodyLowClient(Z2WholeBodyClient):
    def set_positions(self, positions, kps, kds):
        for gid in ALL_GIDS:
            self.set_motor(
                gid,
                mode=0,
                pos=positions[gid],
                kp=kps[gid],
                kd=kds[gid],
            )
        self.setCommand()

    def hold_positions(self, positions, kps, kds, duration=HOLD_BEFORE_RAMP):
        self.set_positions(positions, kps, kds)
        time.sleep(duration)

    def ramp_to_targets(self, start_positions, target_positions, kps, kds,
                        duration=RAMP_DURATION, period=RAMP_PERIOD):
        steps = max(1, int(duration / period))
        for step in range(steps + 1):
            alpha = step / steps
            positions = interpolate_positions(ALL_GIDS, start_positions, target_positions, alpha)
            self.set_positions(positions, kps, kds)
            time.sleep(period)

    def hold_targets_forever(self, target_positions, kps, kds, period=HOLD_PERIOD):
        self.set_positions(target_positions, kps, kds)
        while True:
            time.sleep(period)


def print_target_positions(target_positions):
    print("Target positions:")
    for gid in ALL_GIDS:
        joint_name = GID_TO_JOINT.get(gid, f"Motor{gid}")
        print(f"  gid {gid:2d} ({joint_name:20s}): {target_positions[gid]: .4f} rad")


def main():
    target_positions, stance_kp, stance_kd = build_command_maps(WHOLEBODY_TARGET_STANCE)

    print("Initializing wholebody DDS client...")
    client = Z2WholeBodyLowClient()

    try:
        print("Waiting for current wholebody motor positions...")
        current_positions = wait_for_current_positions(client, timeout=5.0)
        print_positions(current_positions, title="Initial wholebody positions")
        print_target_positions(target_positions)

        print("Holding current positions with stiffness...")
        client.hold_positions(current_positions, stance_kp, stance_kd)

        print(f"Moving to target positions in {RAMP_DURATION:.1f}s...")
        client.ramp_to_targets(
            current_positions,
            target_positions,
            stance_kp,
            stance_kd,
            duration=RAMP_DURATION,
            period=RAMP_PERIOD,
        )
        print("Target positions reached. Holding command. Press Ctrl+C to stop.")
        client.hold_targets_forever(target_positions, stance_kp, stance_kd)
    except (TimeoutError, ValueError) as exc:
        print(f"ERROR: {exc}")
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        client.stop()
        print("Client stopped.")


if __name__ == "__main__":
    main()
