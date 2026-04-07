from robot.end_effectors import (
    EE2_TC,
    lock_tool_changer,
)
from setup import confirm_keys
from .abstracted import go_to_target_height_offset

"""SCREW DRIVER STAND"""
APRIl_TAG_ID_SCREWDRIVER_STAND = 1
SCREWDRIVER_STAND_M = 0.075  # Tool stand entry vertical clearance.
SCREWDRIVER_STAND_AND_TOOL_M = 0.15  # Tool stand exit and raise clearance.


def tool_change_to_screw_driver():
    confirm_keys("MOVE ABOVE TO <SCREW DRIVER>")
    for _ in range(3):
        go_to_target_height_offset(
            april_tag_id=APRIl_TAG_ID_SCREWDRIVER_STAND,
            height=SCREWDRIVER_STAND_M,
        )

    confirm_keys("MOVE DOWN TO <SCREW DRIVER>")
    steps = 5
    height = SCREWDRIVER_STAND_M / steps
    for i in range(steps):
        go_to_target_height_offset(
            april_tag_id=APRIl_TAG_ID_SCREWDRIVER_STAND,
            height=height * (steps - (i + 1)),
            min_segment_time=1.0,  # Smaller segment times.
        )

    confirm_keys("LOCK TOOL AND MOVE ABOVE <SCREW DRIVER>")
    lock_tool_changer(EE2_TC)
    go_to_target_height_offset(
        april_tag_id=APRIl_TAG_ID_SCREWDRIVER_STAND,
        height=SCREWDRIVER_STAND_AND_TOOL_M,
    )
