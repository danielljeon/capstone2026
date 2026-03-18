import time

import serial

from motor_rsbl120 import rsbl120_read_position_step, rsbl120_set_servo_id_nvm
from motor_st3215 import st3215_read_position_step, st3215_set_servo_id_nvm
from robot_arm import JointCal


def rsbl120_id_motor(
    port: str,
    old_id: int,
    new_id: int,
    set_nvm: bool = False,
    print_pos: bool = False,
):
    def __rsbl120_print_pos(joint_cal: JointCal):
        time.sleep(1)
        pos = rsbl120_read_position_step(joint_cal)
        print("Position:", pos)

    # New communication interface.
    comm = serial.Serial(port, 1_000_000, timeout=0.1)

    # Old joint check.
    old_cal = JointCal(name="old_cal", comm=comm, servo_id=old_id)
    if print_pos:
        __rsbl120_print_pos(old_cal)

    # Update servo ID.
    if set_nvm:
        rsbl120_set_servo_id_nvm(old_cal, new_id)
        time.sleep(0.1)

    # New joint check.
    new_cal = JointCal(name="new_cal", comm=comm, servo_id=new_id)
    if print_pos:
        __rsbl120_print_pos(new_cal)

    # Close communication.
    comm.close()


def st3215_id_motor(
    port: str,
    old_id: int,
    new_id: int,
    set_nvm: bool = False,
    print_pos: bool = False,
):
    def __st3215_print_pos(joint_cal: JointCal):
        time.sleep(1)
        pos = st3215_read_position_step(joint_cal)
        print("Position:", pos)

    # New communication interface.
    comm = serial.Serial(port, 1_000_000, timeout=0.1)

    # Old joint check.
    old_cal = JointCal(name="old_cal", comm=comm, servo_id=old_id)
    if print_pos:
        __st3215_print_pos(old_cal)

    # Update servo ID.
    if set_nvm:
        st3215_set_servo_id_nvm(old_cal, new_id)
        time.sleep(0.1)

    # New joint check.
    new_cal = JointCal(name="new_cal", comm=comm, servo_id=new_id)
    if print_pos:
        __st3215_print_pos(new_cal)

    # Close communication.
    comm.close()


if __name__ == "__main__":
    # from constants import RSBL120_PORT
    # rsbl120_id_motor(RSBL120_PORT, 1, 1, False, False)

    # from constants import ST3215_PORT
    # st3215_id_motor(ST3215_PORT, 1, 1, False, False)

    pass
