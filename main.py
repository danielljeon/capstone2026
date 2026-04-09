from constants import *
from main_tasks.abstracted import go_to_optimal_pose
from main_tasks.task_bolt_tighten import bolt_tighten
from main_tasks.task_tool_change_claw import tool_change_to_claw
from main_tasks.task_tool_change_screwdriver import tool_change_to_screw_driver
from main_tasks.task_inchworm import do_inchworm
from recorder import record_targets, record_q_frames
from robot.end_effectors import (
    EE1_TC,
    EE2_TC,
    lock_tool_changer,
    release_tool_changer,
)
from robot.motor_joints import JOINTS
from robot_arm import *
from setup import confirm_keys, set_comms, deinit_comms
from virtualizer import get_active_q, update_tracked_q

# Environment variables load.
load_dotenv()  # Load variables from .env.
URDF_BASE_LINK = os.getenv("URDF_BASE_LINK", "base")
URDF_PATH = os.getenv("URDF_PATH", "./urdf/robot.urdf")


def __startup_zero_pose():
    confirm_keys("LOCK KEYS AND MOVE TO ZERO")

    lock_tool_changer(EE1_TC)

    # Starting positions.
    initial = get_active_q()
    targets = [
        START_POSE,
        JointPose(urdf_joint_angles_active(URDF_BASE_LINK, URDF_PATH)),
    ]
    q_frames = ik_path(
        urdf_base_link=URDF_BASE_LINK,
        urdf_path=URDF_PATH,
        initial_joint_angles_active=initial,
        targets_xyz=targets,
        segment_plans=[None, None],
        dt=IK_DT_S,
        min_segment_time=2.5,
        step_m=0.01,
        smooth_alpha=0.3,
    )
    update_tracked_q(q_frames[-1])
    record_q_frames(q_frames)
    record_targets(targets)
    if not RUN_VIRTUAL:
        execute_q_frames(
            q_frames,
            JOINTS,
            dt=IK_DT_S,
            move_time_ms=int(IK_DT_S * 1000),
            settle_ms=50,
        )


def __go_robot_go():
    confirm_keys(">>>>> GO ROBOT GO!!!!! <<<<<")
    release_tool_changer(EE2_TC)

    go_to_optimal_pose(min_segment_time=5.0)

    # tool_change_to_screw_driver(safety_on=False)
    # go_to_optimal_pose(min_segment_time=5.0)

    # bolt_tighten(safety_on=True) TODO WIP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # go_to_optimal_pose(min_segment_time=5.0)

    # tool_change_to_claw(safety_on=False)
    # go_to_optimal_pose(min_segment_time=5.0)

    # do_inchworm(safety_on=True)


def main():
    can_bus, rsbl120_comm, st3215_comm = None, None, None

    try:
        # Init and assign comms.
        can_bus, rsbl120_comm, st3215_comm = set_comms(
            can_bus_target=not RUN_VIRTUAL,  # Pass False if virtual.
            rsbl120_comm_target=not RUN_VIRTUAL,
            st3215_comm_target=not RUN_VIRTUAL,
        )

        # Initialize each joint.
        for joint in JOINTS:
            if joint.comm is not None:
                joint.init()

        __startup_zero_pose()  # Startup zero pose.

        __go_robot_go()  # Run.

    except KeyboardInterrupt:
        print("\nClosing program...")

    finally:
        # Deinitialize each joint.
        for joint in JOINTS:
            if joint.comm is not None:
                joint.deinit()

        # Close comms.
        deinit_comms(can_bus, rsbl120_comm, st3215_comm)


if __name__ == "__main__":
    main()
