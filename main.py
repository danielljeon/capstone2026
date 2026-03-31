import time

from constants import *
from motion_calcs.motion_path import START_POSE
from robot.motor_joints import JOINTS
from robot_arm import *
from setup import set_comms, deinit_comms


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


def pre_run():
    # Quick start pose.
    for i, j in enumerate(JOINTS):
        if j.comm is not None:
            j.move(START_POSE.q_active[i], 3000)
    time.sleep(3)


def run():
    confirm_keys("RUN")  # Developer type "yes" to continue.

    q_frames = load_q_frames_csv("motion_calcs/motion_20260331_101550.csv")
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )


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
