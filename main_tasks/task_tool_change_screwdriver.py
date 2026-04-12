import time

from robot.end_effectors import EE2_TC, lock_tool_changer, release_tool_changer
from setup import confirm_keys
from .abstracted import go_to_target_height_offset

"""SCREW DRIVER STAND"""
APRIL_TAG_ID_SCREWDRIVER_STAND = 1
APRIL_TAG_SIZE_M_SCREWDRIVER_STAND = 0.04
SCREWDRIVER_STAND_M = 0.075  # Tool stand entry vertical clearance.
SCREWDRIVER_STAND_AND_TOOL_M = 0.15  # Tool stand exit and raise clearance.
SCREWDRIVER_STAND_CALIBRATION_FILE_PATH = (
    "computer_vision_cals/april_tag_cal_screwdriver_stand.csv"
)


def tool_change_to_screw_driver(
    safety_on: bool = True, return_tool: bool = False
):
    if safety_on:
        confirm_keys("MOVE ABOVE TO <SCREW DRIVER>")
    else:
        time.sleep(1)

    for _ in range(3):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_SCREWDRIVER_STAND,
            april_tag_size_m=APRIL_TAG_SIZE_M_SCREWDRIVER_STAND,
            height=SCREWDRIVER_STAND_M,
            april_tag_calibration_filepath=SCREWDRIVER_STAND_CALIBRATION_FILE_PATH,
        )

    if safety_on:
        confirm_keys(
            f"{'LOCK' if return_tool else 'RELEASE'} TC AND MOVE DOWN TO "
            f"<SCREW DRIVER>"
        )
    else:
        time.sleep(1)

    if return_tool:
        lock_tool_changer(EE2_TC)  # Double check locked
    else:
        release_tool_changer(EE2_TC)
    steps = 3
    if return_tool:
        height = SCREWDRIVER_STAND_AND_TOOL_M / steps
    else:
        height = SCREWDRIVER_STAND_M / steps
    for i in range(steps):
        go_to_target_height_offset(
            april_tag_id=APRIL_TAG_ID_SCREWDRIVER_STAND,
            april_tag_size_m=APRIL_TAG_SIZE_M_SCREWDRIVER_STAND,
            height=height * (steps - (i + 1)) - 0.005,
            april_tag_calibration_filepath=SCREWDRIVER_STAND_CALIBRATION_FILE_PATH,
            min_segment_time=1.0,  # Smaller segment times.
        )

    if safety_on:
        confirm_keys(
            f"{'RELEASE' if return_tool else 'LOCK'} TOOL AND MOVE ABOVE "
            f"<SCREW DRIVER>"
        )
    else:
        time.sleep(1)

    if return_tool:
        release_tool_changer(EE2_TC)
    else:
        lock_tool_changer(EE2_TC)
    go_to_target_height_offset(
        april_tag_id=APRIL_TAG_ID_SCREWDRIVER_STAND,
        april_tag_size_m=APRIL_TAG_SIZE_M_SCREWDRIVER_STAND,
        height=SCREWDRIVER_STAND_AND_TOOL_M,
        april_tag_calibration_filepath=SCREWDRIVER_STAND_CALIBRATION_FILE_PATH,
    )
