import os
import time

import numpy as np
from dotenv import load_dotenv

from constants import *
from drivers.motor_rsbl120 import rsbl120_read_position_rad
from drivers.motor_st3215 import st3215_read_position_rad
from motion_calcs.motion_path import START_POSE, OPTIMAL_POSE
from robot.motor_joints import JOINTS
from robot_arm import *
from setup import set_comms, deinit_comms

# Environment variables load.
load_dotenv()  # Load variables from .env.
URDF_BASE_LINK = os.getenv("URDF_BASE_LINK", "base")
URDF_PATH = os.getenv("URDF_PATH", "./urdf/robot.urdf")


def confirm_keys(task: str | None = None):
    while True:
        prompt = (
            f"Type 'yes' to continue to {task}: "
            if task is not None
            else "Type 'yes' to continue"
        )
        resp = input(prompt).strip().lower()
        if resp == "yes":
            break
        print("Please type 'yes' to continue.")
    print("Continuing...")


zero_90_offset = urdf_joint_angles_active(URDF_BASE_LINK, URDF_PATH)
zero_90_offset[0] = -1.57
zero_90_offset_pose = JointPose(zero_90_offset)


def pre_run():
    confirm_keys("PRE")  # Developer type "yes" to continue.

    # Starting positions.
    initial = [
        st3215_read_position_rad(JOINTS[0]),
        rsbl120_read_position_rad(JOINTS[1]),
        rsbl120_read_position_rad(JOINTS[2]),
        rsbl120_read_position_rad(JOINTS[3]),
        rsbl120_read_position_rad(JOINTS[4]),
        st3215_read_position_rad(JOINTS[5]),
    ]
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=initial,
        targets_xyz=[
            START_POSE,
            JointPose(urdf_joint_angles_active(URDF_BASE_LINK, URDF_PATH)),
            OPTIMAL_POSE,
        ],
        segment_plans=[None, None],
        dt=IK_DT_S,
        min_segment_time=2.5,
        step_m=0.01,
        smooth_alpha=0.3,
    )
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )

    time.sleep(3)


def run():
    # while True: TODO
    t_tag_ee = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    # Starting positions.
    initial = [
        st3215_read_position_rad(JOINTS[0]),
        rsbl120_read_position_rad(JOINTS[1]),
        rsbl120_read_position_rad(JOINTS[2]),
        rsbl120_read_position_rad(JOINTS[3]),
        rsbl120_read_position_rad(JOINTS[4]),
        st3215_read_position_rad(JOINTS[5]),
    ]
    ee_pos_world, r_ee_world = fk_ee(URDF_BASE_LINK, URDF_PATH, initial)
    target_world = ee_pos_world + r_ee_world @ t_tag_ee[:3, 3]
    tag_normal_world = r_ee_world @ t_tag_ee[:3, 2]  # Z column directly
    normal_to_target = SegmentPlan(
        mode="fixed_vector",
        vector=tag_normal_world.tolist(),
        orientation_mode="Z",
        target_orientation=tag_normal_world.tolist(),
    )
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=initial,
        targets_xyz=[
            JointPose(initial),
            target_world.tolist(),
        ],
        segment_plans=[normal_to_target],
        dt=IK_DT_S,
        min_segment_time=2.5,
        step_m=0.01,
        smooth_alpha=0.3,
    )

    confirm_keys("VISOR")  # Developer type "yes" to continue.
    try:
        # Animate.
        viser_animate_q(
            urdf_base_link=URDF_BASE_LINK,
            urdf_path=URDF_PATH,
            q_frames=q_frames,
            targets_xyz=[
                JointPose(initial),
                target_world.tolist(),
            ],
            dt=IK_DT_S,
        )
    except KeyboardInterrupt:
        print("Closing visor...")

    confirm_keys("MOVE IK")  # Developer type "yes" to continue.
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )
    time.sleep(5)


if __name__ == "__main__":
    try:
        can_bus, rsbl120_comm, st3215_comm = None, None, None

        try:
            # Init and assign comms.
            can_bus, rsbl120_comm, st3215_comm = set_comms()

            # Initialize each joint.
            for joint in JOINTS:
                if joint.comm is not None:
                    joint.init()

            pre_run()  # Pre-run.
            run()  # Run.

        finally:
            # Deinitialize each joint.
            for joint in JOINTS:
                if joint.comm is not None:
                    joint.deinit()

            # Close comms.
            deinit_comms(can_bus, rsbl120_comm, st3215_comm)

    except KeyboardInterrupt:
        print("\nClosing program...")
