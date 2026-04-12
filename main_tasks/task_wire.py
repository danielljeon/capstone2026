import time

import numpy as np

from constants import TOOL_LOG, TOOL_LOG_PLOT
from robot.end_effectors import (
    EE2_TOOL,
    run_tool_end,
)
from setup import confirm_keys
from .abstracted import go_to_target_height_offset, go_to_target_offset

"""WIRE TASK"""
APRIL_TAG_ID_WIRE_TASK = 4
APRIL_TAG_SIZE_M_WIRE_TASK = 0.04
WIRE_TARGET_VERTICAL_M = 0.08  # Vertical clearance.
WIRE_TARGET_LATERAL_M = 0.05  # Lateral clearance.
WIRE_TASK_CALIBRATION_FILE_PATH = (
    "computer_vision_cals/april_tag_cal_wire_task.csv"
)


def wire_replug(safety_on: bool = True):
    if safety_on:
        confirm_keys("MOVE ABOVE TO <WIRE>")
    else:
        time.sleep(1)

    for _ in range(3):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_WIRE_TASK,
            april_tag_size_m=APRIL_TAG_SIZE_M_WIRE_TASK,
            height=WIRE_TARGET_VERTICAL_M,
            april_tag_calibration_filepath=WIRE_TASK_CALIBRATION_FILE_PATH,
        )

    if safety_on:
        confirm_keys("MOVE DOWN TO <WIRE>")
    else:
        time.sleep(1)

    steps = 5
    height = WIRE_TARGET_VERTICAL_M / steps
    for i in range(steps):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_WIRE_TASK,
            april_tag_size_m=APRIL_TAG_SIZE_M_WIRE_TASK,
            height=height * (steps - (i + 1)),
            april_tag_calibration_filepath=WIRE_TASK_CALIBRATION_FILE_PATH,
        )

    if safety_on:
        confirm_keys("CLOSE CLAW <WIRE>")
    else:
        time.sleep(1)

    run_tool_end(
        hbridge=EE2_TOOL,
        speed=1,
        duration_s=10,
        reverse=False,
        current_limit=350,
        ignore_start_current=True,
        start_current_time_s=0.5,
        plot_playback=TOOL_LOG_PLOT,
        playback_csv_dir=("." if TOOL_LOG else None),
    )

    if safety_on:
        confirm_keys("UNPLUG WIRE <WIRE>")
    else:
        time.sleep(1)

    offset = np.array(
        [
            [1, 0, 0, WIRE_TARGET_LATERAL_M],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    go_to_target_offset(
        april_tag_id=APRIL_TAG_ID_WIRE_TASK,
        april_tag_size_m=APRIL_TAG_SIZE_M_WIRE_TASK,
        offset=offset,
        april_tag_calibration_filepath=WIRE_TASK_CALIBRATION_FILE_PATH,
    )

    if safety_on:
        confirm_keys("REPLUG WIRE <WIRE>")
    else:
        time.sleep(1)

    offset = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    go_to_target_offset(
        april_tag_id=APRIL_TAG_ID_WIRE_TASK,
        april_tag_size_m=APRIL_TAG_SIZE_M_WIRE_TASK,
        offset=offset,
        april_tag_calibration_filepath=WIRE_TASK_CALIBRATION_FILE_PATH,
    )

    if safety_on:
        confirm_keys("OPEN CLAW <WIRE>")
    else:
        time.sleep(1)

    run_tool_end(
        hbridge=EE2_TOOL,
        speed=1,
        duration_s=10,
        reverse=True,
        current_limit=300,
        ignore_start_current=True,
        start_current_time_s=1.0,
        plot_playback=TOOL_LOG_PLOT,
        playback_csv_dir=("." if TOOL_LOG else None),
    )

    if safety_on:
        confirm_keys("MOVE ABOVE TO <WIRE>")
    else:
        time.sleep(1)

    go_to_target_height_offset(
        april_tag_id=APRIL_TAG_ID_WIRE_TASK,
        april_tag_size_m=APRIL_TAG_SIZE_M_WIRE_TASK,
        height=WIRE_TARGET_VERTICAL_M,
        april_tag_calibration_filepath=WIRE_TASK_CALIBRATION_FILE_PATH,
    )
