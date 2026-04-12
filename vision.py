import threading

import cv2
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


_pipeline = None
_detector = None
_intrinsics = None


def _ensure_pipeline():
    global _pipeline, _detector, _intrinsics
    if _pipeline is None:
        _detector = Detector(families="tag36h11")
        _pipeline = start_pipeline()
        _intrinsics = get_realsense_intrinsics(_pipeline)


_latest_img: np.ndarray | None = None
_latest_img_lock = threading.Lock()
_first_frame_ready = threading.Event()
_record_thread: threading.Thread | None = None
_record_stop = threading.Event()


def start_recording(filepath: str) -> None:
    """Start continuous frame capture and write to filepath (mp4).

    Initialises the RealSense pipeline if not already open. The recording thread
    becomes the sole owner of pipeline.wait_for_frames(); all subsequent
    tag_to_robot_tag_detect calls read from the shared frame buffer instead of
    polling the pipeline directly.
    """
    global _record_thread, _latest_img

    _ensure_pipeline()

    _latest_img = None
    _first_frame_ready.clear()
    _record_stop.clear()

    w, h = _intrinsics["width"], _intrinsics["height"]

    def _loop():
        global _latest_img
        writer = cv2.VideoWriter(
            filepath, cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h)
        )
        while not _record_stop.is_set():
            frames = _pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data()).copy()
            writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            with _latest_img_lock:
                _latest_img = img
            _first_frame_ready.set()
        writer.release()

    _record_thread = threading.Thread(target=_loop, daemon=True)
    _record_thread.start()


def stop_recording() -> None:
    """Signal the recording thread to finish and wait for the file to flush."""
    _record_stop.set()
    if _record_thread is not None:
        _record_thread.join()


def tag_to_robot_tag_detect(
    april_tag_id: int, april_tag_size_m: float, calibration_filepath: str
) -> np.ndarray | None:
    _ensure_pipeline()

    # If the recording thread is running it owns wait_for_frames(); read the
    # latest frame it has already captured rather than polling the pipeline.
    img = None
    if _record_thread is not None and _record_thread.is_alive():
        _first_frame_ready.wait()
        with _latest_img_lock:
            img = _latest_img  # already a copy; safe to read outside the lock

    transform, _, _ = detect_tag_zeroed(
        _pipeline,
        _intrinsics,
        april_tag_id,
        april_tag_size_m,
        _detector,
        calibration_filepath,
        img,
    )
    if transform is not None:
        transform = tag_to_ee(transform)
    return transform
