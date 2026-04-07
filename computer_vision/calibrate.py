"""
>>> python calibrate.py 3 0.04 file_path_example.csv  # tag 3, size, file path
"""

import argparse

from pupil_apriltags import Detector

from april_tag_realsense import (
    start_pipeline,
    get_realsense_intrinsics,
    calibrate_zero,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Lock or unlock a tool changer."
    )
    parser.add_argument("tag_id", type=int, help="april tag id to target")
    parser.add_argument("size_m", type=float, help="April tag size in meters")
    parser.add_argument(
        "file_path", type=str, help="file path to save calibration to (.csv)"
    )
    args = parser.parse_args()

    detector = Detector(families="tag36h11")
    pipeline = start_pipeline()
    intrinsics = get_realsense_intrinsics(pipeline)

    calibrate_zero(
        save_file_path=args.file_path,
        pipeline=pipeline,
        intrinsics=intrinsics,
        tag_id=args.tag_id,
        tag_size_m=args.size_m,
        detector=detector,
    )
