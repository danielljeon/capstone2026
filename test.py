from setup import set_comms, deinit_comms


def main():
    can_bus, rsbl120_comm, st3215_comm = None, None, None

    try:
        # Init and assign comms.
        can_bus, rsbl120_comm, st3215_comm = set_comms(
            can_bus_target=False,  # TODO: Set to True and uncomment below.
            rsbl120_comm_target=False,  # TODO: Set to True and uncomment below.
            st3215_comm_target=False,  # TODO: Set to True and uncomment below.
        )

        """Single time call functions (does not require init calls)"""
        # import time
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
        # import time
        # from robot.motor_joints import JOINTS
        #
        # for joint in JOINTS:
        #     joint.move(position_rad=0, move_time_ms=500)

    finally:
        deinit_comms(can_bus, rsbl120_comm, st3215_comm)
