import os

from dotenv import load_dotenv

from constants import IK_DT_S
from motion_path import MOVE_1_TARGETS, MOVE_1_PLANS
from robot_arm import *

# Environment variables load.
load_dotenv()  # Load variables from .env.
URDF_BASE_LINK = f'.{os.getenv("URDF_BASE_LINK", "base")}'
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


if __name__ == "__main__":
    try:
        confirm_keys("Calculate IK")  # Developer type "yes" to continue.

        # Calculate IK.
        q_frames = ik_path(
            urdf_base_link=URDF_BASE_LINK,
            urdf_path=URDF_PATH,
            initial_joint_angles_active=urdf_joint_angles_active(
                URDF_BASE_LINK, URDF_PATH
            ),
            targets_xyz=MOVE_1_TARGETS,
            segment_plans=MOVE_1_PLANS,
            dt=IK_DT_S,
            min_segment_time=2.5,
            step_m=0.01,
            smooth_alpha=0.3,
        )
        p = save_q_frames_now_csv(q_frames)
        q_frames = load_q_frames_csv(p)

        confirm_keys("Animation")  # Developer type "yes" to continue.

        # Animate.
        viser_animate_q(
            urdf_base_link=URDF_BASE_LINK,
            urdf_path=URDF_PATH,
            q_frames=q_frames,
            targets_xyz=MOVE_1_TARGETS,
            dt=IK_DT_S,
        )

    except KeyboardInterrupt:
        print("\nClosing program...")
