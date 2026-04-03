import os
from typing import Iterable, Sequence

import numpy as np
from dotenv import load_dotenv

from constants import IK_DT_S, OPTIMAL_POSE, ZERO_POSE_EE_POS
from robot_arm import *

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


def ik_and_animate(
    initial_q: Sequence[float],
    targets_xyz: Iterable[Sequence[float] | JointPose],
    segment_plans: list[SegmentPlan | None] | None,
    dt: float,
    animate_viser: bool = True,
) -> np.ndarray:
    # Calculate IK.
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=initial_q,
        targets_xyz=targets_xyz,
        segment_plans=segment_plans,
        dt=dt,
        min_segment_time=4.0,
        step_m=0.01,
        smooth_alpha=0.3,
    )
    p = save_q_frames_now_csv(q_frames)
    q_frames = load_q_frames_csv(p)

    if animate_viser:
        try:
            # Animate.
            viser_animate_q(
                urdf_base_link=URDF_BASE_LINK,
                urdf_path=URDF_PATH,
                q_frames=q_frames,
                targets_xyz=MOVE_1_TARGETS,
                dt=IK_DT_S,
            )
        except KeyboardInterrupt:
            print("Closing visor...")

    return q_frames


if __name__ == "__main__":
    MOVE_1_INITIAL_Q = urdf_joint_angles_active(URDF_BASE_LINK, URDF_PATH)

    T_TOOL = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0.35],
            [0, 0, 1, ZERO_POSE_EE_POS[2] - 0.167],
            [0, 0, 0, 1],
        ]
    )
    TOOL_POSITION = np.asarray(T_TOOL[:3, 3], dtype=float)
    TOOL_ORIENTATION = np.asarray(T_TOOL[:3, :3], dtype=float)
    TOOL_ABOVE_OFFSET = 0.16

    t_tool_error = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    target_down = (
        TOOL_POSITION + TOOL_ORIENTATION @ t_tool_error[:3, 3]
    ).tolist()
    target_up = target_down.copy()
    target_up[2] += TOOL_ABOVE_OFFSET

    # Targets in meters.
    MOVE_1_TARGETS = [
        OPTIMAL_POSE,
        target_up,
        TOOL_POSITION,
        target_down,
    ]

    # Targets in meters.
    MOVE_2_TARGETS = [
        target_up,
        OPTIMAL_POSE,
    ]

    _, r_ee_world = fk_ee(URDF_BASE_LINK, URDF_PATH, MOVE_1_TARGETS[0].q_active)
    tag_normal_world = r_ee_world @ T_TOOL[:3, 2]  # Z column directly
    target_rot_world = r_ee_world @ T_TOOL[:3, :3]
    OPTIMAL_MOVE_SEGMENT_PLAN = SegmentPlan(
        mode="fixed_rotation",
        vector=tag_normal_world.tolist(),
        orientation_mode="full",
        target_orientation=tag_normal_world.tolist(),
        target_rotation=target_rot_world.tolist(),
    )

    # One plan per segment between initial angles and each target.
    MOVE_1_PLANS = [
        None,
        OPTIMAL_MOVE_SEGMENT_PLAN,
        OPTIMAL_MOVE_SEGMENT_PLAN,
        OPTIMAL_MOVE_SEGMENT_PLAN,
    ]

    # One plan per segment between initial angles and each target.
    MOVE_2_PLANS = [
        OPTIMAL_MOVE_SEGMENT_PLAN,
        OPTIMAL_MOVE_SEGMENT_PLAN,
    ]

    try:
        q_frames = ik_and_animate(
            initial_q=MOVE_1_INITIAL_Q,
            targets_xyz=MOVE_1_TARGETS,
            segment_plans=MOVE_1_PLANS,
            dt=IK_DT_S,
            # animate_viser=False,
        )
        MOVE_2_INITIAL_Q = q_frames[-1]  # Use last q for next move IK calc.
        ik_and_animate(
            initial_q=MOVE_2_INITIAL_Q,
            targets_xyz=MOVE_2_TARGETS,
            segment_plans=MOVE_2_PLANS,
            dt=IK_DT_S,
            # animate_viser=False,
        )

    except KeyboardInterrupt:
        print("\nClosing program...")
