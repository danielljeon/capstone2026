"""Quick test: lock or unlock a tool changer.

>>> python demo_tool_changer.py lock 1
>>> python demo_tool_changer.py unlock 2
"""

import argparse
import time

from robot.end_effectors import (
    EE1_TC,
    EE2_TC,
    lock_tool_changer,
    release_tool_changer,
)
from setup import set_comms, deinit_comms

TC_MAP = {1: EE1_TC, 2: EE2_TC}


def main():
    parser = argparse.ArgumentParser(
        description="Lock or unlock a tool changer."
    )
    parser.add_argument(
        "action",
        choices=["lock", "unlock"],
        help="lock or unlock the tool changer",
    )
    parser.add_argument(
        "tc", type=int, choices=[1, 2], help="tool changer number (1 or 2)"
    )
    args = parser.parse_args()

    tc = TC_MAP[args.tc]

    can_bus, rsbl120_comm, st3215_comm = None, None, None
    try:
        can_bus, rsbl120_comm, st3215_comm = set_comms(
            can_bus_target=True,
            rsbl120_comm_target=False,
            st3215_comm_target=False,
        )

        if args.action == "lock":
            print(f"Locking tool changer {args.tc}...")
            lock_tool_changer(tc)
        else:
            print(f"Unlocking tool changer {args.tc}...")
            release_tool_changer(tc)

        time.sleep(1)
        print("Done.")

    except KeyboardInterrupt:
        print("Closing...")

    finally:
        deinit_comms(can_bus, rsbl120_comm, st3215_comm)


if __name__ == "__main__":
    main()
