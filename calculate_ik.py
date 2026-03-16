import os

from dotenv import load_dotenv

from motion_path import targets, plans
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


if __name__ == "__main__":
    try:
        dt = 0.02

        confirm_keys("Calculate IK")  # Developer type "yes" to continue.

        # Calculate IK.
        q_frames = ik_path(
            urdf_base_link=URDF_BASE_LINK,
            urdf_path=URDF_PATH,
            initial_joint_angles_active=urdf_joint_angles_active(
                URDF_BASE_LINK, URDF_PATH
            ),
            targets_xyz=targets,
            segment_plans=plans,
            dt=dt,
            min_segment_time=2.5,
            step_m=0.01,
            smooth_alpha=0.3,
        )
        p = save_q_frames_now_csv(q_frames)
        q_frames = load_q_frames_csv(p)

        confirm_keys("Animation")  # Developer type "yes" to continue.

        # Animate.
        animate_q(
            urdf_base_link=URDF_BASE_LINK,
            urdf_path=URDF_PATH,
            q_frames=q_frames,
            targets_xyz=targets,
            show_frames=True,
            frame_scale=0.05,
            frame_stride=1,
        )

    except KeyboardInterrupt:
        print("\nClosing program...")
