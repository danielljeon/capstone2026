import numpy as np

from constants import *
from recorder import record_targets, record_q_frames
from robot.motor_joints import JOINTS
from robot_arm import *
from virtualizer import get_active_q, update_tracked_q
from vision import tag_to_robot_tag_detect


def go_to_optimal_pose(min_segment_time: float = 3.0):
    initial_q = get_active_q()
    targets = [OPTIMAL_POSE]
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=initial_q,
        targets_xyz=targets,
        segment_plans=[SegmentPlan("free")],
        dt=IK_DT_S,
        min_segment_time=min_segment_time,
        step_m=0.01,
        smooth_alpha=0.3,
    )
    update_tracked_q(q_frames[-1])
    record_q_frames(q_frames)
    record_targets(targets)
    if RUN_VIRTUAL:
        execute_q_frames(
            q_frames,
            JOINTS,
            dt=IK_DT_S,
            move_time_ms=int(IK_DT_S * 1000),
            settle_ms=50,
        )


def go_to_target_height_offset(
    april_tag_id: int,
    april_tag_calibration_filepath: str,
    height: float,
    min_segment_time: float = 3.0,
):
    initial_q = get_active_q()
    t_april_tag = tag_to_robot_tag_detect(
        april_tag_id, APRIL_TAG_SIZE_M_STANDARD, april_tag_calibration_filepath
    )
    offset = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, height],
            [0, 0, 0, 1],
        ]
    )
    q_frames = ik_relative_from_q(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_q=initial_q,
        t_target=t_april_tag,
        dt=IK_DT_S,
        min_segment_time=min_segment_time,
        step_m=0.01,
        smooth_alpha=0.3,
        offset=offset,
    )
    update_tracked_q(q_frames[-1])
    record_q_frames(q_frames)
    # TODO: Reimplements internal logic of ik_relative_from_q in order to record
    #  targets.
    temp_transform = calculate_t_relative_from_q(
        URDF_BASE_LINK, URDF_PATH, initial_q, t_april_tag, offset
    )
    temp_target_pos = temp_transform[:3, 3]
    temp_targets = [temp_target_pos.tolist()]
    record_targets(temp_targets)
    if RUN_VIRTUAL:
        execute_q_frames(
            q_frames,
            JOINTS,
            dt=IK_DT_S,
            move_time_ms=int(IK_DT_S * 1000),
            settle_ms=50,
        )
