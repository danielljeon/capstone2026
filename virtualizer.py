"""Handles real or virtual joint angle tracking.

In real hardware joint angles will be read from current positions.

In virtual mode, this module tracks the last joint angles allowing for some
level of motion simulation.
"""

from constants import RUN_VIRTUAL
from drivers.motor_rsbl120 import rsbl120_read_position_rad
from drivers.motor_st3215 import st3215_read_position_rad
from robot.motor_joints import JOINTS

tracked_q = []


def get_active_q() -> list[float]:
    if RUN_VIRTUAL:
        global tracked_q
        return tracked_q

    else:
        n = 10
        readings = [
            [
                st3215_read_position_rad(JOINTS[0]),
                rsbl120_read_position_rad(JOINTS[1]),
                rsbl120_read_position_rad(JOINTS[2]),
                rsbl120_read_position_rad(JOINTS[3]),
                rsbl120_read_position_rad(JOINTS[4]),
                st3215_read_position_rad(JOINTS[5]),
            ]
            for _ in range(n)
        ]
        avg_q = [sum(r[i] for r in readings) / n for i in range(6)]
        return avg_q


def update_tracked_q(active_q: list[float]):
    global tracked_q
    if len(tracked_q) == len(active_q):
        tracked_q = active_q.copy()
    else:
        raise RuntimeWarning(
            f"Active Q size mismatch, expected {len(tracked_q)}, received "
            f"{len(active_q)}"
        )
