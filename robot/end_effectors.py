import time

from drivers.motor_pwm_node_constants import db
from drivers.motor_pwm_node_hbridge import HBridge, hbridge_drive, hbridge_coast
from drivers.motor_pwm_node_servo import (
    pwm_node_servo_send_move,
)
from robot_arm import *

EE1_TC = JointCal(
    name="ee1_tool_changer",
    comm=None,
    servo_id=1,
    sign=1,
    hardware_zero=0,
)
EE1_TOOL = HBridge(
    bus=None,
    channel_in1=2,
    channel_in2=3,
    channel_enable=4,
    current_message=db.get_message_by_name("current_sense_01"),
)
EE2_TC = JointCal(
    name="ee2_tool_changer",
    comm=None,
    servo_id=5,
    sign=1,
    hardware_zero=0,
)
EE2_TOOL = HBridge(
    bus=None,
    channel_in1=6,
    channel_in2=7,
    channel_enable=8,
    current_message=db.get_message_by_name("current_sense_02"),
)


def release_tool_changer(cal: JointCal):
    pwm_node_servo_send_move(cal, 0)


def lock_tool_changer(cal: JointCal):
    pwm_node_servo_send_move(cal, 1.5707)  # 90 deg.


def run_tool_end(
    hbridge: HBridge,
    speed: float,
    duration_s: float,
    reverse: bool,
    current_limit: float | None = None,
    ignore_start_current: bool = False,
    start_current_time_s: float = 0.1,
):
    try:
        if current_limit is None:
            hbridge_drive(
                hbridge.bus, hbridge, speed, duration_s, reverse=reverse
            )

        else:
            start = time.time()

            hbridge_drive(hbridge.bus, hbridge, speed, 0, reverse=reverse)

            # Ignore current for start_current_time_s if enabled.
            if ignore_start_current:
                while time.time() - start < start_current_time_s:
                    pass

            # Check for current limit.
            while (time.time() - start) < duration_s:
                msg = hbridge.bus.recv(timeout=0.1)

                if (
                    msg
                    and msg.arbitration_id == hbridge.current_message.frame_id
                ):
                    decoded = hbridge.current_message.decode(msg.data)
                    current = decoded["current_mA"]

                    if current > current_limit:
                        break

    finally:
        hbridge_coast(hbridge.bus, hbridge)
