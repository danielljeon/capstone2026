"""
AprilTag detection with Intel RealSense D455f.
Outputs 4x4 camera-to-tag transform, with optional zeroed relative transform.
"""

import time
from pathlib import Path

import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector

CALIBRATION_FILE = Path("zero_transform.npy")


def get_realsense_intrinsics(pipeline: rs.pipeline) -> dict:
    profile = pipeline.get_active_profile()
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    i = stream.get_intrinsics()
    return {
        "fx": i.fx,
        "fy": i.fy,
        "cx": i.ppx,
        "cy": i.ppy,
        "width": i.width,
        "height": i.height,
    }


def start_pipeline() -> rs.pipeline:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    pipeline.start(config)
    return pipeline


def detect_tag(
    pipeline: rs.pipeline,
    intrinsics: dict,
    tag_id: int,
    tag_size_m: float,
    detector: Detector,
) -> np.ndarray | None:
    """
    Grab one frame and return the 4x4 camera-to-tag transform for the given
    tag_id.

    Returns:
        None if the tag is not detected.
    """
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if not color:
        return None

    img = np.asanyarray(color.get_data())
    gray = np.dot(img[..., :3], [0.299, 0.587, 0.114]).astype(np.uint8)

    camera_params = (
        intrinsics["fx"],
        intrinsics["fy"],
        intrinsics["cx"],
        intrinsics["cy"],
    )

    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size_m,
    )

    for det in detections:
        if det.tag_id != tag_id:
            continue
        T = np.eye(4)
        T[:3, :3] = det.pose_R
        T[:3, 3] = det.pose_t.flatten()
        return T

    return None


def calibrate_zero(
    pipeline: rs.pipeline,
    intrinsics: dict,
    tag_id: int,
    tag_size_m: float,
    detector: Detector,
    n_frames: int = 30,
    save_path: Path = CALIBRATION_FILE,
) -> np.ndarray:
    """
    Average T_cam_tag over n_frames and save as the zero transform.

    Returns:
        Averaged 4x4 zero transform.
    """
    print(f"Capturing {n_frames} frames for zero calibration...")
    transforms = []

    while len(transforms) < n_frames:
        T = detect_tag(pipeline, intrinsics, tag_id, tag_size_m, detector)
        if T is not None:
            transforms.append(T)
        else:
            print(
                f"  Tag not detected, retrying... ({len(transforms)}/{n_frames})"
            )
            time.sleep(0.05)

    # Average rotation via mean of rotation matrices (close enough for small variance)
    R_mean = np.mean([T[:3, :3] for T in transforms], axis=0)
    # Re-orthogonalise via SVD
    U, _, Vt = np.linalg.svd(R_mean)
    R_ortho = U @ Vt

    t_mean = np.mean([T[:3, 3] for T in transforms], axis=0)

    T_zero = np.eye(4)
    T_zero[:3, :3] = R_ortho
    T_zero[:3, 3] = t_mean

    np.save(save_path, T_zero)
    print(f"Zero saved to {save_path}")
    return T_zero


def load_zero(path: Path = CALIBRATION_FILE) -> np.ndarray | None:
    if path.exists():
        return np.load(path)
    return None


def apply_zero(T_current: np.ndarray, T_zero: np.ndarray) -> np.ndarray:
    """
    Returns:
        T_current expressed relative to T_zero: T_rel = T_zero_inv @ T_current
    """
    return np.linalg.inv(T_zero) @ T_current
