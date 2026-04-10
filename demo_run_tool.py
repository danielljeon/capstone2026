"""Quick test: run a tool end forward or backward.

>>> python demo_run_tool.py 2 forward
>>> python demo_run_tool.py 2 backward --plot --csv-dir .  # local dir `.`
"""

import argparse
import time

from robot.end_effectors import (
    EE1_TOOL,
    EE2_TOOL,
    run_tool_end,
)
from setup import set_comms, deinit_comms

TOOL_MAP = {1: EE1_TOOL, 2: EE2_TOOL}
CURRENT_LIMIT = {1: 450.0, 2: 450.0}


def main():
    parser = argparse.ArgumentParser(
        description="Run a tool end forward or backward."
    )
    parser.add_argument(
        "tool", type=int, choices=[1, 2], help="tool number (1 or 2)"
    )
    parser.add_argument(
        "direction",
        choices=["forward", "backward"],
        help="run direction",
    )
    parser.add_argument(
        "--plot", action="store_true", help="enable playback plotting"
    )
    parser.add_argument(
        "--csv-dir",
        type=str,
        default=None,
        help="directory to save playback CSV (omit to skip)",
    )
    args = parser.parse_args()

    tool = TOOL_MAP[args.tool]
    forward = args.direction == "forward"
    current_limit = CURRENT_LIMIT[args.tool]

    # Build optional kwargs only when requested.
    run_kwargs = {}
    if args.plot:
        run_kwargs["plot_playback"] = True
    if args.csv_dir is not None:
        run_kwargs["playback_csv_dir"] = args.csv_dir

    can_bus, rsbl120_comm, st3215_comm = None, None, None
    try:
        can_bus, rsbl120_comm, st3215_comm = set_comms(
            can_bus_target=True,
            rsbl120_comm_target=False,
            st3215_comm_target=False,
        )

        print(
            f"Running tool {args.tool} {'forward' if forward else 'backward'} "
            f"(current limit {current_limit} mA)..."
        )
        run_tool_end(
            tool,
            1,
            25,
            forward,
            current_limit,
            ignore_start_current=True,
            start_current_time_s=0.5,
            **run_kwargs,
        )
        time.sleep(1)
        print("Done.")

    except KeyboardInterrupt:
        print("Closing...")

    finally:
        deinit_comms(can_bus, rsbl120_comm, st3215_comm)


if __name__ == "__main__":
    main()
