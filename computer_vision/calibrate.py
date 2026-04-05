from pupil_apriltags import Detector

from april_tag_realsense import (
    start_pipeline,
    get_realsense_intrinsics,
    calibrate_zero,
)

if __name__ == "__main__":
    TAG_ID = 1
    TAG_SIZE_M = 0.04  # metres.

    detector = Detector(families="tag36h11")
    pipeline = start_pipeline()
    intrinsics = get_realsense_intrinsics(pipeline)

    calibrate_zero(pipeline, intrinsics, TAG_ID, TAG_SIZE_M, detector)
