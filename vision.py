import numpy as np
from pupil_apriltags import Detector

from computer_vision.april_tag_realsense import (
    start_pipeline,
    get_realsense_intrinsics,
    detect_tag_zeroed,
)

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
)


def tag_to_ee(t: np.ndarray) -> np.ndarray:
    """Remap a zeroed tag transform into the EE frame."""
    t = T_APRIL_TAG_TO_EE_POS @ t @ T_APRIL_TAG_TO_EE_ROT
    t[0, 3] *= -1
    t[2, 3] *= -1
    d = np.diag([-1.0, 1.0, -1.0])
    t[:3, :3] = d @ t[:3, :3] @ d
    return t


def tool_stand_detect(
    april_tag_id: int, april_tag_size_m: float
) -> np.ndarray | None:
    detector = Detector(families="tag36h11")
    pipeline = start_pipeline()
    intrinsics = get_realsense_intrinsics(pipeline)

    transform, _, _ = detect_tag_zeroed(
        pipeline, intrinsics, april_tag_id, april_tag_size_m, detector
    )
    if transform is not None:
        transform = tag_to_ee(transform)
    return transform
