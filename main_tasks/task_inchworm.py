import time

import numpy as np

from constants import *
from recorder import record_targets, record_q_frames
from robot.motor_joints import JOINTS
from robot_arm import *
from setup import confirm_keys
from virtualizer import get_active_q, update_tracked_q
from .abstracted import go_to_target_height_offset, go_to_target_offset

"""INCH WORM"""
APRIL_TAG_ID_INCHWORM = 1
APRIL_TAG_SIZE_M_INCHWORM = 0.04
INCHWORM_TARGET_VERTICAL_M = 0.15  # Vertical clearance.
INCHWORM_TARGET_LATERAL_M = 0.02  # Lateral clearance.
INCHWORM_CALIBRATION_FILE_PATH = (
    "computer_vision_cals/april_tag_cal_inchworm.csv"
)


def __april_tag_clearance_loop(height_m: float):
    go_to_target_height_offset(
        april_tag_id=APRIL_TAG_ID_INCHWORM,
        april_tag_size_m=APRIL_TAG_SIZE_M_INCHWORM,
        height=height_m,
        april_tag_calibration_filepath=INCHWORM_CALIBRATION_FILE_PATH,
    )


def __april_tag_on_target_loop(lateral_m: float, height_m: float):
    offset = np.array(
        [
            [1, 0, 0, lateral_m],
            [0, 1, 0, 0],
            [0, 0, 1, height_m],
            [0, 0, 0, 1],
        ]
    )
    go_to_target_offset(
        april_tag_id=APRIL_TAG_ID_INCHWORM,
        april_tag_size_m=APRIL_TAG_SIZE_M_INCHWORM,
        offset=offset,
        april_tag_calibration_filepath=INCHWORM_CALIBRATION_FILE_PATH,
    )


def __go_to_inchworm_pose():
    inchworm_pose_1 = JointPose([0, 0.55, 0.50, -2.60, -0.55, -1.39])
    inchworm_pose_2 = JointPose([-1.87, 0.55, 0.50, -2.60, -0.55, -1.39])

    initial_q = get_active_q()
    targets = [
        inchworm_pose_1,  # Move to clear the first (old) handrail laterally.
        inchworm_pose_2,  # Sweep towards the second (new) handrail laterally.
    ]
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=initial_q,
        targets_xyz=targets,
        segment_plans=[None, None],
        dt=IK_DT_S,
        min_segment_time=5.0,
        step_m=0.01,
        smooth_alpha=0.3,
    )
    update_tracked_q(q_frames[-1])
    record_q_frames(q_frames)
    record_targets(targets)
    if not RUN_VIRTUAL:
        execute_q_frames(
            q_frames,
            JOINTS,
            dt=IK_DT_S,
            move_time_ms=int(IK_DT_S * 1000),
            settle_ms=50,
        )


def do_inchworm(safety_on: bool = True):
    if safety_on:
        confirm_keys("MOVE ABOVE TO <INCH WORM>")
    else:
        time.sleep(1)

    __go_to_inchworm_pose()

    # if safety_on:
    #     confirm_keys("MOVE TO NEW BAR <INCH WORM>")
    # else:
    #     time.sleep(1)
    #
    # for _ in range(3):
    #     __april_tag_clearance_loop(INCHWORM_TARGET_VERTICAL_M)
    #
    # steps = 3
    # height = INCHWORM_TARGET_VERTICAL_M / steps
    # lateral = INCHWORM_TARGET_LATERAL_M / steps
    # for i in range(steps):
    #     __april_tag_on_target_loop(
    #         height * (steps - (i + 1)), lateral * (steps - (i + 1))
    #     )

    # if safety_on:
    #     confirm_keys("CLAMP CLAW (2) ON NEW BAR <INCH WORM>")
    # else:
    #     time.sleep(1)
    #
    # run_tool_end(
    #     EE2_TOOL,
    #     speed=1,
    #     duration_s=45,
    #     reverse=False,  # Close claw.
    #     current_limit=350,
    #     ignore_start_current=True,
    #     start_current_time_s=0.5,
    # )
    #
    # if safety_on:
    #     confirm_keys("OPEN CLAW (1) ON OLD BAR AND MOVE AWAY <INCH WORM>")
    # else:
    #     time.sleep(1)
    #
    # run_tool_end(
    #     EE2_TOOL,
    #     speed=1,
    #     duration_s=45,
    #     reverse=True,  # Open claw.
    #     current_limit=350,
    #     ignore_start_current=True,
    #     start_current_time_s=0.5,
    # )
    #
    # if safety_on:
    #     confirm_keys("MOVE TO NEW MIRRORED ZERO <INCH WORM>")
    # else:
    #     time.sleep(1)
