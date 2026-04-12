""" Detects AprilTags via RealSense camera, printing pose transforms to the
terminal or displaying a live annotated overlay with optional video recording.

>>> python demo_april_tag.py  # headless
>>> python demo_april_tag.py --gui  # live overlay
>>> python demo_april_tag.py --gui --tag 3  # target tag 3
>>> python demo_april_tag.py --gui --calibration zero_transform.csv  # cal file
"""

import argparse
import datetime

import cv2
import numpy as np
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation

from computer_vision.april_tag_realsense import (
    start_pipeline,
    get_realsense_intrinsics,
    detect_tag_zeroed,
)
from vision import tag_to_ee


def print_transform(T: np.ndarray, label: str = "Transform") -> None:
    t = T[:3, 3]
    yaw, pitch, roll = Rotation.from_matrix(T[:3, :3]).as_euler(
        "ZYX", degrees=True
    )
    print(f"\n{label}")
    print(f"  Pos (m):   x={t[0]:+.4f}  y={t[1]:+.4f}  z={t[2]:+.4f}")
    print(f"  RPY (deg): R={roll:+.2f}  P={pitch:+.2f}  Y={yaw:+.2f}")
    for row in T[:3, :3]:
        print(f"    [{row[0]:+.4f}  {row[1]:+.4f}  {row[2]:+.4f}]")


def run_headless(april_tag_id: int = 1):
    from vision import tag_to_robot_tag_detect

    while True:
        T = tag_to_robot_tag_detect(april_tag_id=april_tag_id)
        if T is not None:
            print_transform(T, "Tool Stand")


def run_gui(
    flip: bool = False, april_tag_id: int = 1, april_tag_size_m: float = 0.04
):
    detector = Detector(families="tag36h11")
    pipeline = start_pipeline()
    intrinsics = get_realsense_intrinsics(pipeline)

    cam_mtx = np.array(
        [
            [intrinsics["fx"], 0, intrinsics["cx"]],
            [0, intrinsics["fy"], intrinsics["cy"]],
            [0, 0, 1],
        ]
    )

    recording = False
    writer = None
    win = "AprilTag - Tool Stand"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960, 540)

    try:
        while True:
            T, frame, det = detect_tag_zeroed(
                pipeline,
                intrinsics,
                april_tag_id,
                april_tag_size_m,
                detector,
                calibration_filepath=None,
            )
            if frame is None:
                continue

            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Draw tag overlay before flipping (detection coords match original frame)
            if det is not None:
                pts = det.corners.astype(int)
                for i in range(4):
                    cv2.line(
                        frame,
                        tuple(pts[i]),
                        tuple(pts[(i + 1) % 4]),
                        (0, 255, 0),
                        2,
                    )
                cx, cy = int(det.center[0]), int(det.center[1])
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"id {det.tag_id}",
                    (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                rvec, _ = cv2.Rodrigues(det.pose_R)
                tvec = det.pose_t.reshape(3, 1)
                cv2.drawFrameAxes(
                    frame, cam_mtx, np.zeros(4), rvec, tvec, 0.03, 2
                )

                # Label each axis tip
                axis_tips = np.float32(
                    [[0.03, 0, 0], [0, 0.03, 0], [0, 0, 0.03]]
                )
                proj, _ = cv2.projectPoints(
                    axis_tips, rvec, tvec, cam_mtx, np.zeros(4)
                )
                for (name, colour), pt in zip(
                    [
                        ("X", (0, 0, 255)),
                        ("Y", (0, 255, 0)),
                        ("Z", (255, 0, 0)),
                    ],
                    proj.reshape(-1, 2),
                ):
                    cv2.putText(
                        frame,
                        name,
                        (int(pt[0]) + 5, int(pt[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        colour,
                        1,
                        cv2.LINE_AA,
                    )

            if flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            h, w = frame.shape[:2]

            # Text overlays (drawn after flip so they stay readable)
            if T is not None:
                ee = tag_to_ee(T.copy())
                t = ee[:3, 3]
                yaw, pitch, roll = Rotation.from_matrix(ee[:3, :3]).as_euler(
                    "ZYX", degrees=True
                )
                cv2.putText(
                    frame,
                    f"x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} m",
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"R={roll:+.1f} P={pitch:+.1f} Y={yaw:+.1f} deg",
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    frame,
                    "Tag not detected",
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

            if recording:
                cv2.circle(frame, (w - 20, 20), 8, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    "REC",
                    (w - 60, 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )
                writer.write(frame)

            # Help bar with dark background strip
            bar_text = f"[Q]uit  [R]ec  [S]nap  [F]lip{'*' if flip else ''}  tag={april_tag_id}"
            bar_h = 28
            overlay = frame[h - bar_h : h, :].copy()
            cv2.rectangle(frame, (0, h - bar_h), (w, h), (0, 0, 0), -1)
            cv2.addWeighted(
                overlay,
                0.3,
                frame[h - bar_h : h, :],
                0.7,
                0,
                frame[h - bar_h : h, :],
            )
            cv2.putText(
                frame,
                bar_text,
                (10, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow(win, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("r"):
                if not recording:
                    fname = f"apriltag_rec_{datetime.datetime.now():%Y%m%d_%H%M%S}.avi"
                    writer = cv2.VideoWriter(
                        fname, cv2.VideoWriter_fourcc(*"XVID"), 30.0, (w, h)
                    )
                    recording = True
                    print(f"Recording -> {fname}")
                else:
                    recording = False
                    writer.release()
                    writer = None
                    print("Recording stopped.")

            elif key == ord("s"):
                fname = (
                    f"apriltag_snap_{datetime.datetime.now():%Y%m%d_%H%M%S}.png"
                )
                cv2.imwrite(fname, frame)
                print(f"Saved -> {fname}")

            elif key == ord("f"):
                flip = not flip
                print(f"Flip {'on' if flip else 'off'}")

    finally:
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gui", action="store_true", help="Live camera overlay"
    )
    parser.add_argument(
        "--flip",
        action="store_true",
        help="Rotate image 180 (upside-down camera)",
    )
    parser.add_argument(
        "--tag",
        type=int,
        default=1,
        help="AprilTag ID to detect (default: 1)",
    )
    parser.add_argument(
        "--calibration",
        type=str,
        default=None,
        help="Calibration file path to apply (default: None)",
    )
    args = parser.parse_args()

    if args.gui:
        run_gui(flip=args.flip, april_tag_id=args.tag)
    else:
        run_headless(april_tag_id=args.tag)
