from robot.end_effectors import (
    EE2_TOOL,
    run_tool_end,
)
from setup import confirm_keys
from .abstracted import go_to_target_height_offset

"""BOLT TASK"""
APRIl_TAG_ID_BOLT_TASK = 3
BOLT_TASK_M = 0.05  # Task jig entry vertical clearance.
BOLT_TASK_CALIBRATION_FILE_PATH = (
    "computer_vision_cals/april_tag_cal_bolt_task.csv"
)


def bolt_tighten():
    confirm_keys("MOVE ABOVE TO <BOLT>")
    for _ in range(3):
        go_to_target_height_offset(
            april_tag_id=APRIl_TAG_ID_BOLT_TASK,
            height=BOLT_TASK_M,
            april_tag_calibration_filepath=BOLT_TASK_CALIBRATION_FILE_PATH,
        )

    confirm_keys("MOVE DOWN TO <BOLT>")
    steps = 5
    height = BOLT_TASK_M / steps
    for i in range(steps):
        go_to_target_height_offset(
            april_tag_id=APRIl_TAG_ID_BOLT_TASK,
            height=height * (steps - (i + 1)),
            april_tag_calibration_filepath=BOLT_TASK_CALIBRATION_FILE_PATH,
        )

    confirm_keys("TIGHTEN <BOLT>")
    run_tool_end(
        hbridge=EE2_TOOL,
        speed=1,
        duration_s=10,
        reverse=False,
        current_limit=350,
        ignore_start_current=True,
        start_current_time_s=0.5,
        plot_playback=True,
        playback_csv_dir=".",
    )

    confirm_keys("MOVE ABOVE <BOLT>")
    go_to_target_height_offset(
        april_tag_id=APRIl_TAG_ID_BOLT_TASK,
        height=BOLT_TASK_M,
        april_tag_calibration_filepath=BOLT_TASK_CALIBRATION_FILE_PATH,
    )
