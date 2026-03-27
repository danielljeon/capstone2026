import time

from constants import *
from drivers.motor_joints import joint_cals
from drivers.motor_rsbl120 import (
    rsbl120_open_comm,
    rsbl120_close_comm,
)
from drivers.motor_st3215 import (
    st3215_open_comm,
    st3215_close_comm,
)
from robot_arm import *


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

        confirm_keys("Motion")  # Developer type "yes" to continue.

        # Open comms.
        rsbl120_comm = rsbl120_open_comm(RSBL120_PORT)
        st3215_comm = st3215_open_comm(ST3215_PORT)

        # Assign comms objects for each joint.
        for joint in joint_cals:
            if "rsbl120" in joint.name:
                joint.comm = rsbl120_comm
            elif "st3215" in joint.name:
                joint.comm = st3215_comm

        # Initialize each joint.
        for joint in joint_cals:
            if joint.comm is not None:
                joint.init()

        # Quick Zero.
        for joint in joint_cals:
            if joint.comm is not None:
                joint.move(0, 1000)
        time.sleep(1)

        try:
            # Execute IK.
            execute_q_frames(
                q_frames, joint_cals, dt=dt, move_time_ms=30, settle_ms=50
            )
            pass

        finally:
            # Deinitialize each joint.
            for joint in joint_cals:
                if joint.comm is not None:
                    joint.deinit()

            # Close comms.
            rsbl120_close_comm(rsbl120_comm)
            st3215_close_comm(st3215_comm)

    except KeyboardInterrupt:
        print("\nClosing program...")
