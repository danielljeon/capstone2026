import re

import can
import numpy as np
import serial

from constants import *
from drivers.motor_pwm_node_servo import (
    pwm_node_servo_open_comm,
    pwm_node_servo_close_comm,
)
from drivers.motor_rsbl120 import rsbl120_open_comm, rsbl120_close_comm
from drivers.motor_st3215 import st3215_open_comm, st3215_close_comm
from robot.end_effectors import EE1_TC, EE1_TOOL, EE2_TC, EE2_TOOL
from robot.motor_joints import JOINTS


def __init_comms(
    can_bus_target: bool = True,
    rsbl120_comm_target: bool = True,
    st3215_comm_target: bool = True,
) -> tuple[can.BusABC, serial.Serial, serial.Serial]:
    can_bus, rsbl120_comm, st3215_comm = None, None, None

    if can_bus_target:
        can_bus = pwm_node_servo_open_comm(
            PWM_NODE_SERVO_INTERFACE, PWM_NODE_SERVO_CHANNEL
        )
    if rsbl120_comm_target:
        rsbl120_comm = rsbl120_open_comm(RSBL120_PORT)
    if st3215_comm_target:
        st3215_comm = st3215_open_comm(ST3215_PORT)

    return can_bus, rsbl120_comm, st3215_comm


def set_comms(
    can_bus_target: bool = True,
    rsbl120_comm_target: bool = True,
    st3215_comm_target: bool = True,
) -> tuple[can.BusABC, serial.Serial, serial.Serial]:
    can_bus, rsbl120_comm, st3215_comm = __init_comms(
        can_bus_target, rsbl120_comm_target, st3215_comm_target
    )

    if can_bus_target:
        for ee_tc in [EE1_TC, EE2_TC]:
            ee_tc.comm = can_bus
        for tool in [EE1_TOOL, EE2_TOOL]:
            tool.bus = can_bus

    for joint in JOINTS:
        if rsbl120_comm_target and "rsbl120" in joint.name:
            joint.comm = rsbl120_comm
        if st3215_comm_target and "st3215" in joint.name:
            joint.comm = st3215_comm

    return can_bus, rsbl120_comm, st3215_comm


def deinit_comms(
    can_bus: can.BusABC | None,
    rsbl120_comm: serial.Serial | None,
    st3215_comm: serial.Serial | None,
) -> None:
    if can_bus is not None:
        pwm_node_servo_close_comm(can_bus)
    if rsbl120_comm is not None:
        rsbl120_close_comm(rsbl120_comm)
    if st3215_comm is not None:
        st3215_close_comm(st3215_comm)


def load_diff_from_file(filepath: str) -> np.ndarray:
    """Load a 4x4 matrix from a text file and return as numpy array.

    Args:
        filepath: Path to text file containing 4x4 matrix rows.

    Examples:
        ```
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        ```

    Returns:
        4x4 numpy array.
    """
    with open(filepath, "r") as f:
        content = f.read()

    # Extract all rows of the form [a, b, c, d]
    rows = re.findall(r"\[([^\]]+)\]", content)
    assert len(rows) == 4, f"Expected 4 rows, got {len(rows)}"

    matrix = np.array([[float(x) for x in row.split(",")] for row in rows])
    assert matrix.shape == (4, 4), f"Expected 4x4, got {matrix.shape}"

    return matrix
