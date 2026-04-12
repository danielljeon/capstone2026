"""
AprilTag detection with Intel RealSense D455f.
Outputs 4x4 camera-to-tag transform, with optional zeroed relative transform.
"""

import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector


def get_realsense_intrinsics(pipeline: rs.pipeline) -> dict:
    profile = pipeline.get_active_profile()
    stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    i = stream.get_intrinsics()

    camera_matrix = np.array(
        [
            [i.fx, 0, i.ppx],
            [0, i.fy, i.ppy],
            [0, 0, 1],
        ]
    )
    dist_coeffs = np.array(i.coeffs)

    # Compute once — reuse for every frame
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (i.width, i.height), 0
    )

    return {
        "fx": new_camera_matrix[0, 0],
        "fy": new_camera_matrix[1, 1],
        "cx": new_camera_matrix[0, 2],
        "cy": new_camera_matrix[1, 2],
        "width": i.width,
        "height": i.height,
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "new_camera_matrix": new_camera_matrix,
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
    img: np.ndarray | None = None,
) -> tuple[np.ndarray | None, np.ndarray | None, object | None]:
    """Grab one frame and detect the given tag.

    If *img* is supplied (RGB, HxWx3) the pipeline is not polled — used when
    a recording thread already owns frame capture.

    Returns:
         (T_cam_tag, rgb_frame, detection), any element may be None.
    """
    if img is None:
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return None, None, None
        img = np.asanyarray(color.get_data())

    img = cv2.undistort(
        img,
        intrinsics["camera_matrix"],
        intrinsics["dist_coeffs"],
        None,
        intrinsics["new_camera_matrix"],
    )

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
        transform = np.eye(4)
        transform[:3, :3] = det.pose_R
        transform[:3, 3] = det.pose_t.flatten()
        return transform, img, det

    return None, img, None


def calibrate_zero(
    save_file_path: str,
    pipeline: rs.pipeline,
    intrinsics: dict,
    tag_id: int,
    tag_size_m: float,
    detector: Detector,
    n_frames: int = 30,
) -> np.ndarray:
    """Average T_cam_tag over n_frames and save as the zero transform.

    Returns:
        Averaged 4x4 zero transform.
    """
    print(f"Capturing {n_frames} frames for zero calibration...")
    transforms = []

    while len(transforms) < n_frames:
        transform, _, _ = detect_tag(
            pipeline, intrinsics, tag_id, tag_size_m, detector
        )
        if transform is not None:
            transforms.append(transform)
        else:
            print(
                f"  Tag not detected, retrying... ({len(transforms)}/{n_frames})"
            )
            time.sleep(0.05)

    # Average rotation via mean of rotation matrices (close enough for small
    # variance)
    R_mean = np.mean([T[:3, :3] for T in transforms], axis=0)
    # Re-orthogonalise via SVD
    U, _, Vt = np.linalg.svd(R_mean)
    R_ortho = U @ Vt

    t_mean = np.mean([T[:3, 3] for T in transforms], axis=0)

    t_zero = np.eye(4)
    t_zero[:3, :3] = R_ortho
    t_zero[:3, 3] = t_mean

    p = Path(save_file_path)
    np.savetxt(p, t_zero, delimiter=",")
    print(f"Zero saved to {save_file_path}")
    return t_zero


def load_zero(file_path: str) -> np.ndarray | None:
    p = Path(file_path)
    if p.exists():
        return np.loadtxt(p, delimiter=",")
    return None


def apply_zero(t_current: np.ndarray, t_zero: np.ndarray) -> np.ndarray:
    """
    Returns:
        t_current expressed relative to t_zero: T_rel = t_zero_inv @ t_current
    """
    return np.linalg.inv(t_zero) @ t_current


def detect_tag_zeroed(
    pipeline: rs.pipeline,
    intrinsics: dict,
    tag_id: int,
    tag_size_m: float,
    detector: Detector,
    calibration_filepath: str | None,
    img: np.ndarray | None = None,
) -> tuple[np.ndarray | None, np.ndarray | None, object | None]:
    """Detect tag and apply zero calibration.

    Returns (T_zeroed, rgb_frame, detection) - any element may be None.
    """
    transform, frame, det = detect_tag(
        pipeline, intrinsics, tag_id, tag_size_m, detector, img
    )

    if calibration_filepath is not None:
        t_zero = load_zero(calibration_filepath)
        transform = apply_zero(transform, t_zero)

    return transform, frame, det
