from constants import URDF_BASE_LINK, URDF_PATH
from drivers.motor_rsbl120 import (
    DEFAULT_STEP_PER_RAD as RSBL120_DEFAULT_STEP_PER_RAD,
)
from drivers.motor_rsbl120 import rsbl120_read_position_step
from drivers.motor_st3215 import (
    DEFAULT_STEP_PER_RAD as ST3215_DEFAULT_STEP_PER_RAD,
)
from drivers.motor_st3215 import st3215_read_position_step
from robot.motor_joints import JOINTS
from robot_arm import step_to_rad, fk_ee
from setup import set_comms, deinit_comms


def main():
    can_bus, rsbl120_comm, st3215_comm = None, None, None

    try:
        # Init and assign comms.
        can_bus, rsbl120_comm, st3215_comm = set_comms(
            can_bus_target=False,
            rsbl120_comm_target=True,
            st3215_comm_target=True,
        )

        q = []
        q_raw = []
        for joint in JOINTS:
            if joint.comm is not None:
                if "rsbl120" in joint.name:
                    raw = rsbl120_read_position_step(joint)
                    q_raw.append(raw)
                    q.append(
                        step_to_rad(
                            raw,
                            joint,
                            RSBL120_DEFAULT_STEP_PER_RAD,
                        )
                    )
                elif "st3215" in joint.name:
                    raw = st3215_read_position_step(joint)
                    q_raw.append(raw)
                    q.append(
                        step_to_rad(
                            raw,
                            joint,
                            ST3215_DEFAULT_STEP_PER_RAD,
                        )
                    )

        print("q_active (step):")
        print(q_raw)

        print("q_active (rad):")
        print(q)

        if None not in q:
            ee_pos, ee_rot = fk_ee(URDF_BASE_LINK, URDF_PATH, q_active=q)
            print("EE Position:")
            print(ee_pos)
            print("EE Rotation:")
            print(ee_rot)

    except KeyboardInterrupt:
        print("\nClosing program...")

    finally:
        deinit_comms(can_bus, rsbl120_comm, st3215_comm)


if __name__ == "__main__":
    main()
