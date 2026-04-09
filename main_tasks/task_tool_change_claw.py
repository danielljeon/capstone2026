import time

from robot.end_effectors import EE2_TC, lock_tool_changer
from setup import confirm_keys
from .abstracted import go_to_target_height_offset

"""CLAW STAND"""
APRIL_TAG_ID_CLAW_STAND = 2
APRIL_TAG_SIZE_M_BOLT_TASK = 0.03
CLAW_STAND_M = 0.075  # Tool stand entry vertical clearance.
CLAW_STAND_AND_TOOL_M = 0.15  # Tool stand exit and raise clearance.
CLAW_STAND_CALIBRATION_FILE_PATH = (
    "computer_vision_cals/april_tag_cal_claw_stand.csv"
)


def tool_change_to_claw(safety_on: bool = True):
    if safety_on:
        confirm_keys("MOVE ABOVE TO <CLAW>")
    else:
        time.sleep(1)

    for _ in range(3):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_CLAW_STAND,
            april_tag_size_m=APRIL_TAG_SIZE_M_BOLT_TASK,
            height=CLAW_STAND_M,
            april_tag_calibration_filepath=CLAW_STAND_CALIBRATION_FILE_PATH,
        )

    if safety_on:
        confirm_keys("MOVE DOWN TO <CLAW>")
    else:
        time.sleep(1)

    steps = 3
    height = CLAW_STAND_M / steps
    for i in range(steps):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_CLAW_STAND,
            april_tag_size_m=APRIL_TAG_SIZE_M_BOLT_TASK,
            height=height * (steps - (i + 1)) - 0.005,
            april_tag_calibration_filepath=CLAW_STAND_CALIBRATION_FILE_PATH,
            min_segment_time=1.0,  # Smaller segment times.
        )

    if safety_on:
        confirm_keys("LOCK TOOL AND MOVE ABOVE <CLAW>")
    else:
        time.sleep(1)

    lock_tool_changer(EE2_TC)
    go_to_target_height_offset(
        april_tag_id=APRIL_TAG_ID_CLAW_STAND,
        april_tag_size_m=APRIL_TAG_SIZE_M_BOLT_TASK,
        height=CLAW_STAND_AND_TOOL_M,
        april_tag_calibration_filepath=CLAW_STAND_CALIBRATION_FILE_PATH,
    )
