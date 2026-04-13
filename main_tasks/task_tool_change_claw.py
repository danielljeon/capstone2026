import time

from robot.end_effectors import EE2_TC, lock_tool_changer, release_tool_changer
from setup import confirm_keys
from .abstracted import go_to_target_height_offset

"""CLAW STAND"""
APRIL_TAG_ID_CLAW_STAND = 2
APRIL_TAG_SIZE_M_BOLT_TASK = 0.03
CLAW_STAND_M = 0.075  # Tool stand entry vertical clearance.
CLAW_STAND_AND_TOOL_M = 0.13  # Tool stand exit and raise clearance.
CLAW_STAND_CALIBRATION_FILE_PATH = (
    "computer_vision_cals/april_tag_cal_claw_stand.csv"
)


def tool_change_to_claw(safety_on: bool = True, return_tool: bool = False):
    if safety_on:
        confirm_keys("MOVE ABOVE TO <CLAW>")
    else:
        time.sleep(1)

    if return_tool:
        height = CLAW_STAND_AND_TOOL_M
    else:
        height = CLAW_STAND_M
    for _ in range(3):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_CLAW_STAND,
            april_tag_size_m=APRIL_TAG_SIZE_M_BOLT_TASK,
            height=height,
            april_tag_calibration_filepath=CLAW_STAND_CALIBRATION_FILE_PATH,
        )

    if safety_on:
        confirm_keys(
            f"{'LOCK' if return_tool else 'RELEASE'} TC AND MOVE DOWN TO "
            f"<CLAW>"
        )
    else:
        time.sleep(1)

    if return_tool:
        lock_tool_changer(EE2_TC)  # Double check locked
    else:
        release_tool_changer(EE2_TC)

    steps = 3
    if return_tool:
        height = CLAW_STAND_AND_TOOL_M / steps
    else:
        height = CLAW_STAND_M / steps
    for i in range(steps):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_CLAW_STAND,
            april_tag_size_m=APRIL_TAG_SIZE_M_BOLT_TASK,
            height=height * (steps - (i + 1)) - 0.005,
            # Lock orientation only when close.
            lock_full_orientation=True if i > 0 else False,
            april_tag_calibration_filepath=CLAW_STAND_CALIBRATION_FILE_PATH,
            min_segment_time=1.0,  # Smaller segment times.
        )
        time.sleep(1)

    if safety_on:
        confirm_keys(
            f"{'RELEASE' if return_tool else 'LOCK'} TOOL AND MOVE ABOVE "
            f"<CLAW>"
        )
    else:
        time.sleep(1)

    if return_tool:
        release_tool_changer(EE2_TC)
    else:
        lock_tool_changer(EE2_TC)

    go_to_target_height_offset(
        april_tag_id=APRIL_TAG_ID_CLAW_STAND,
        april_tag_size_m=APRIL_TAG_SIZE_M_BOLT_TASK,
        height=CLAW_STAND_AND_TOOL_M,
        april_tag_calibration_filepath=CLAW_STAND_CALIBRATION_FILE_PATH,
    )
