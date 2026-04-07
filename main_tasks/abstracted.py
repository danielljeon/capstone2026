import numpy as np

from constants import *
from drivers.motor_rsbl120 import rsbl120_read_position_rad
from drivers.motor_st3215 import st3215_read_position_rad
from robot.motor_joints import JOINTS
from robot_arm import *
from vision import tag_to_robot_tag_detect


def go_to_target_height_offset(april_tag_id: int, height: float):
    initial_q = [
        st3215_read_position_rad(JOINTS[0]),
        rsbl120_read_position_rad(JOINTS[1]),
        rsbl120_read_position_rad(JOINTS[2]),
        rsbl120_read_position_rad(JOINTS[3]),
        rsbl120_read_position_rad(JOINTS[4]),
        st3215_read_position_rad(JOINTS[5]),
    ]
    t_april_tag = tag_to_robot_tag_detect(
        april_tag_id, APRIL_TAG_SIZE_M_STANDARD
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
        min_segment_time=4.0,
        step_m=0.01,
        smooth_alpha=0.3,
        offset=offset,
    )
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )
