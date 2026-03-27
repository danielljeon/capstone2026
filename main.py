import time

from constants import *
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


if __name__ == "__main__":
    print_serial_ports()

    try:
        dt = 0.02

        # Load IK calculations.
        q_frames = load_q_frames_csv(IK_FILE_CSV)

        can_bus, rsbl120_comm, st3215_comm = None, None, None

        try:
            # Init and assign comms.
            can_bus, rsbl120_comm, st3215_comm = set_comms()

            # Initialize each joint.
            for joint in JOINTS:
                if joint.comm is not None:
                    joint.init()

            # Quick Zero.
            for joint in JOINTS:
                if joint.comm is not None:
                    joint.move(0, 1000)
            time.sleep(1)

            confirm_keys("Motion")  # Developer type "yes" to continue.

            # Execute IK.
            execute_q_frames(
                q_frames, JOINTS, dt=dt, move_time_ms=30, settle_ms=50
            )
            pass

        finally:
            # Deinitialize each joint.
            for joint in JOINTS:
                if joint.comm is not None:
                    joint.deinit()

            # Close comms.
            deinit_comms(can_bus, rsbl120_comm, st3215_comm)

    except KeyboardInterrupt:
        print("\nClosing program...")
