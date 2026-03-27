import time

from drivers.motor_rsbl120 import rsbl120_read_position_step
from drivers.motor_st3215 import st3215_read_position_step
from robot.motor_joints import JOINTS
from robot_arm import JointCal
from setup import set_comms, deinit_comms

CAN_BUS_TARGET = False  # TODO: Set to True and uncomment below.
RSBL120_COMM_TARGET = False  # TODO: Set to True and uncomment below.
ST3215_COMM_TARGET = False  # TODO: Set to True and uncomment below.


def report_q():
    def __rsbl120_print_pos(joint_cal: JointCal):
        time.sleep(1)
        pos = rsbl120_read_position_step(joint_cal)
        print(f"{joint.name} ({joint.servo_id}) position:", pos)

    def __st3215_print_pos(joint_cal: JointCal):
        time.sleep(1)
        pos = st3215_read_position_step(joint_cal)
        print(f"{joint.name} ({joint.servo_id}) position:", pos)

    # Init and assign comms.
    can_bus, rsbl120_comm, st3215_comm = set_comms(
        can_bus_target=False,
        rsbl120_comm_target=RSBL120_COMM_TARGET,
        st3215_comm_target=ST3215_COMM_TARGET,
    )

    try:
        for joint in JOINTS:
            if joint.comm is not None:
                if "rsbl120" in joint.name:
                    __rsbl120_print_pos(joint)
                elif "st3215" in joint.name:
                    __st3215_print_pos(joint)

    finally:
        deinit_comms(can_bus, rsbl120_comm, st3215_comm)


def main():
    can_bus, rsbl120_comm, st3215_comm = None, None, None

    try:
        # Init and assign comms.
        can_bus, rsbl120_comm, st3215_comm = set_comms(
            can_bus_target=CAN_BUS_TARGET,
            rsbl120_comm_target=RSBL120_COMM_TARGET,
            st3215_comm_target=ST3215_COMM_TARGET,
        )

        """Single time call functions (does not require init calls)"""
        # from robot.end_effectors import (
        #     EE1_TC,
        #     EE2_TC,
        #     EE1_TOOL,
        #     EE2_TOOL,
        #     lock_tool_changer,
        #     release_tool_changer,
        #     run_tool_end,
        # )
        #
        # lock_tool_changer(EE1_TC)
        # lock_tool_changer(EE2_TC)
        # time.sleep(1)
        # run_tool_end(EE1_TOOL, 1, 25, True, 350.0)
        # run_tool_end(EE2_TOOL, 1, 25, True, 350.0)
        # time.sleep(1)
        # run_tool_end(EE1_TOOL, 1, 25, False, 350.0)
        # run_tool_end(EE2_TOOL, 1, 25, False, 350.0)
        # time.sleep(1)
        # release_tool_changer(EE1_TC)
        # release_tool_changer(EE2_TC)

        """Dynamic function calls"""
        # for joint in JOINTS:
        #     joint.move(position_rad=0, move_time_ms=500)

    finally:
        deinit_comms(can_bus, rsbl120_comm, st3215_comm)


if __name__ == "__main__":
    report_q()
    main()
