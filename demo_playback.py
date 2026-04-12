"""Replay a recorder logged q_frames CSV using matplotlib or viser.

>>> python demo_playback.py recording.csv
>>> python demo_playback.py recording.csv --viewer matplotlib
>>> python demo_playback.py recording.csv --xyz-file targets_xyz.csv --pose-file targets_pose.csv
>>> python demo_playback.py recording.csv --viewer viser --port 8080
"""

import argparse

from constants import URDF_BASE_LINK, URDF_PATH
from recorder import playback, playback_viser


def main():
    parser = argparse.ArgumentParser(
        description="Replay a recorded q_frames CSV."
    )
    parser.add_argument(
        "q_frames_file",
        type=str,
        help="path to the q_frames CSV file",
    )
    parser.add_argument(
        "--viewer",
        choices=["viser", "matplotlib"],
        default="viser",
        help="viewer to use for playback (default: viser)",
    )
    parser.add_argument(
        "--xyz-file",
        type=str,
        default=None,
        help="path to targets XYZ CSV file (optional)",
    )
    parser.add_argument(
        "--pose-file",
        type=str,
        default=None,
        help="path to targets pose CSV file (optional)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="viser server port (default: 8080, viser only)",
    )
    args = parser.parse_args()

    if args.viewer == "viser":
        print(f"Starting viser playback on port {args.port}...")
        playback_viser(
            URDF_BASE_LINK,
            URDF_PATH,
            args.q_frames_file,
            xyz_file=args.xyz_file,
            pose_file=args.pose_file,
            port=args.port,
        )
    else:
        print("Starting matplotlib playback...")
        playback(
            URDF_BASE_LINK,
            URDF_PATH,
            args.q_frames_file,
            xyz_file=args.xyz_file,
            pose_file=args.pose_file,
        )


if __name__ == "__main__":
    main()
