import numpy as np

from constants import *
from drivers.motor_rsbl120 import rsbl120_read_position_rad
from drivers.motor_st3215 import st3215_read_position_rad
from robot.end_effectors import (
    EE1_TC,
    EE2_TC,
    lock_tool_changer,
    release_tool_changer,
)
from robot.motor_joints import JOINTS
from robot_arm import *
from setup import deinit_comms, set_comms
from vision import tool_stand_detect

# Environment variables load.
load_dotenv()  # Load variables from .env.
URDF_BASE_LINK = os.getenv("URDF_BASE_LINK", "base")
URDF_PATH = os.getenv("URDF_PATH", "./urdf/robot.urdf")


TOOL_STAND_HEIGHT_OFFSET_M = 0.1


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
    confirm_keys("LOCK KEYS")  # Developer type "yes" to continue.

    lock_tool_changer(EE1_TC)
    lock_tool_changer(EE2_TC)

    confirm_keys("PRE")  # Developer type "yes" to continue.

    # Starting positions.
    initial = [
        st3215_read_position_rad(JOINTS[0]),
        rsbl120_read_position_rad(JOINTS[1]),
        rsbl120_read_position_rad(JOINTS[2]),
        rsbl120_read_position_rad(JOINTS[3]),
        rsbl120_read_position_rad(JOINTS[4]),
        st3215_read_position_rad(JOINTS[5]),
    ]
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=initial,
        targets_xyz=[
            START_POSE,
            JointPose(urdf_joint_angles_active(URDF_BASE_LINK, URDF_PATH)),
        ],
        segment_plans=[None, None],
        dt=IK_DT_S,
        min_segment_time=2.5,
        step_m=0.01,
        smooth_alpha=0.3,
    )
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )


def __go_to_optimal():
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=urdf_joint_angles_active(
            URDF_BASE_LINK, URDF_PATH
        ),
        targets_xyz=[OPTIMAL_POSE],
        segment_plans=[SegmentPlan("free")],
        dt=IK_DT_S,
        min_segment_time=4.0,
        step_m=0.01,
        smooth_alpha=0.3,
    )
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )


def __go_to_tool_stand_above():
    initial_q = [
        st3215_read_position_rad(JOINTS[0]),
        rsbl120_read_position_rad(JOINTS[1]),
        rsbl120_read_position_rad(JOINTS[2]),
        rsbl120_read_position_rad(JOINTS[3]),
        rsbl120_read_position_rad(JOINTS[4]),
        st3215_read_position_rad(JOINTS[5]),
    ]
    t_april_tag = tool_stand_detect()
    offset = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, TOOL_STAND_HEIGHT_OFFSET_M],
            [0, 0, 0, 1],
        ]
    )
    q_frames = ik_relative_from_q(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_q=initial_q,
        t_target=t_april_tag,
        animate=True,
        dt=IK_DT_S,
        min_segment_time=4.0,
        step_m=0.01,
        smooth_alpha=0.3,
        offset=offset,
    )
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )


def __go_z_tool_stand_height(go_down: bool = False):
    initial_q = [
        st3215_read_position_rad(JOINTS[0]),
        rsbl120_read_position_rad(JOINTS[1]),
        rsbl120_read_position_rad(JOINTS[2]),
        rsbl120_read_position_rad(JOINTS[3]),
        rsbl120_read_position_rad(JOINTS[4]),
        st3215_read_position_rad(JOINTS[5]),
    ]
    direction = -1 if go_down else 1
    target = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, direction * TOOL_STAND_HEIGHT_OFFSET_M],
            [0, 0, 0, 1],
        ]
    )
    q_frames = ik_relative_from_q(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_q=initial_q,
        t_target=target,
        animate=True,
        dt=IK_DT_S,
        min_segment_time=4.0,
        step_m=0.01,
        smooth_alpha=0.3,
        offset=None,
    )
    execute_q_frames(
        q_frames,
        JOINTS,
        dt=IK_DT_S,
        move_time_ms=int(IK_DT_S * 1000),
        settle_ms=50,
    )
    return target


def run():
    confirm_keys("RELEASE TOOL END")  # Developer type "yes" to continue.
    release_tool_changer(EE2_TC)

    confirm_keys("GO TO OPTIMAL POSE")  # Developer type "yes" to continue.
    __go_to_optimal()

    confirm_keys("MOVE TO ABOVE TARGET")
    __go_to_tool_stand_above()

    confirm_keys("MOVE DOWN FIXED HEIGHT TO TARGET")
    __go_z_tool_stand_height(go_down=True)

    confirm_keys("LOCK TOOL")
    lock_tool_changer(EE2_TC)

    confirm_keys("MOVE UP FIXED HEIGHT FROM TARGET")
    __go_z_tool_stand_height(go_down=False)


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
