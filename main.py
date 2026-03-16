import time

from constants import *
from drivers.motor_rsbl120 import (
    send_move,
    read_position_step,
    open_actuator_serial_comm,
    close_actuator_comm,
)
from robot_arm import *


def example_motor_init(j: JointCal):
    print(f"Motor {j.name} ({j.servo_id}) POS {read_position_step(j)}")


joint_cals = [
    JointCal(
        name="st3215-12_1",
        comm=None,
        servo_id=2,
        hardware_zero=0.0,
        software_zero_rad=0.0,
        sign=+1,
        rad_min=-0.0,
        rad_max=+0.0,
        init_func=example_motor_init,
        move_func=send_move,
        deinit_func=motor_deinit,
    ),
    JointCal(
        name="rsbl120-24_1",
        comm=None,
        servo_id=2,
        hardware_zero=3179,
        software_zero_rad=0.0,
        sign=-1,
        rad_min=-2.44346,
        rad_max=0.0,
        init_func=example_motor_init,
        move_func=send_move,
        deinit_func=motor_deinit,
    ),
    JointCal(
        name="rsbl120-24_2",
        comm=None,
        servo_id=3,
        hardware_zero=3115,
        software_zero_rad=0.0,
        sign=+1,
        rad_min=-0.174533,
        rad_max=3.31613,
        init_func=example_motor_init,
        move_func=send_move,
        deinit_func=motor_deinit,
    ),
    JointCal(
        name="rsbl120-24_3",
        comm=None,
        servo_id=4,
        hardware_zero=3144,
        software_zero_rad=0.0,
        sign=-1,
        rad_min=-3.31613,
        rad_max=0.174533,
        init_func=example_motor_init,
        move_func=send_move,
        deinit_func=motor_deinit,
    ),
    JointCal(
        name="rsbl120-24_4",
        comm=None,
        servo_id=5,
        hardware_zero=1256,
        software_zero_rad=0.0,
        sign=-1,
        rad_min=0.0,
        rad_max=+2.44346,
        init_func=example_motor_init,
        move_func=send_move,
        deinit_func=motor_deinit,
    ),
    JointCal(
        name="st3215-12_2",
        comm=None,
        servo_id=3,
        hardware_zero=0.0,
        software_zero_rad=0.0,
        sign=+1,
        rad_min=-0.0,
        rad_max=+0.0,
        init_func=example_motor_init,
        move_func=send_move,
        deinit_func=motor_deinit,
    ),
]


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
        rsbl120_comm = open_actuator_serial_comm(RSBL120_PORT)
        st3215_comm = open_actuator_serial_comm(ST3215_PORT)

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
            close_actuator_comm(rsbl120_comm)
            close_actuator_comm(st3215_comm)

    except KeyboardInterrupt:
        print("\nClosing program...")
