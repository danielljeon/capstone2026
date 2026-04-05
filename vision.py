import numpy as np
from pupil_apriltags import Detector

from computer_vision.april_tag_realsense import (
    start_pipeline,
    get_realsense_intrinsics,
    detect_tag_zeroed,
)

TOOL_STAND_TAG_ID = 1
TOOL_STAND_TAG_SIZE_M = 0.04  # metres.
T_APRIL_TAG_TO_EE_POS = np.array(
    [
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ]
)
T_APRIL_TAG_TO_EE_ROT = np.array(
    [
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1],
    ]
)  # Rx(-90) * Rz(-90)


def tool_stand_detect() -> np.ndarray | None:
    detector = Detector(families="tag36h11")
    pipeline = start_pipeline()
    intrinsics = get_realsense_intrinsics(pipeline)

    transform = detect_tag_zeroed(
        pipeline, intrinsics, TOOL_STAND_TAG_ID, TOOL_STAND_TAG_SIZE_M, detector
    )
    if transform is not None:
        transform = T_APRIL_TAG_TO_EE_POS @ transform @ T_APRIL_TAG_TO_EE_ROT

        # Fix axis inversions
        transform[0, 3] *= -1
        transform[2, 3] *= -1

        # Fix rotation inversions
        d = np.diag([-1.0, 1.0, -1.0])
        transform[:3, :3] = d @ transform[:3, :3] @ d

    return transform
