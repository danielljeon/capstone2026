import csv
import os
import time
from datetime import datetime

import matplotlib.pyplot as plt

from constants import RUN_VIRTUAL
from drivers.motor_pwm_node_constants import db
from drivers.motor_pwm_node_hbridge import HBridge, hbridge_drive, hbridge_coast
from drivers.motor_pwm_node_servo import (
    pwm_node_servo_send_move,
)
from robot_arm import JointCal

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
    if RUN_VIRTUAL:
        print("release_tool_changer() COMPLETE RUN VIRTUAL")
        return
    pwm_node_servo_send_move(cal, 0)


def lock_tool_changer(cal: JointCal):
    if RUN_VIRTUAL:
        print("lock_tool_changer() COMPLETE RUN VIRTUAL")
        return
    pwm_node_servo_send_move(cal, 1.5707)  # 90 deg.


def run_tool_end(
    hbridge: HBridge,
    speed: float,
    duration_s: float,
    reverse: bool,
    current_limit: float | None = None,
    ignore_start_current: bool = False,
    start_current_time_s: float = 0.1,
    plot_playback: bool = False,
    playback_csv_dir: str | None = None,
):
    if RUN_VIRTUAL:
        print("run_tool_end() COMPLETE RUN VIRTUAL")
        return

    times = []
    currents = []

    try:
        if current_limit is None:
            hbridge_drive(
                hbridge.bus, hbridge, speed, duration_s, reverse=reverse
            )

        else:
            start = time.time()

            hbridge_drive(hbridge.bus, hbridge, speed, 0, reverse=reverse)

            if ignore_start_current:
                while time.time() - start < start_current_time_s:
                    pass

            # Flush stale CAN messages before monitoring.
            while hbridge.bus.recv(timeout=0) is not None:
                pass

            while (time.time() - start) < duration_s:
                msg = hbridge.bus.recv(timeout=0.1)

                if (
                    msg
                    and msg.arbitration_id == hbridge.current_message.frame_id
                ):
                    decoded = hbridge.current_message.decode(msg.data)
                    current = decoded["current_mA"]

                    times.append(time.time() - start)
                    currents.append(current)

                    if current > current_limit:
                        break

    finally:
        hbridge_coast(hbridge.bus, hbridge)

    if playback_csv_dir is not None and times:
        limit_str = (
            f"{int(current_limit)}mA"
            if current_limit is not None
            else "nolimit"
        )
        filename = (
            f"tool_end_{limit_str}_"
            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        filepath = os.path.join(playback_csv_dir, filename)

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "current_mA", "current_limit_mA"])
            for t, c in zip(times, currents):
                writer.writerow([t, c, current_limit])

    if plot_playback and times:
        plt.figure()
        plt.plot(times, currents, label="Current (mA)")
        plt.axhline(y=current_limit, color="r", linestyle="--", label="Limit")
        plt.xlabel("Time (s)")
        plt.ylabel("Current (mA)")
        plt.legend()
        plt.title("Tool End Current")
        plt.show()
