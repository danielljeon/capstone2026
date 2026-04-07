import csv
import os
from datetime import datetime

import numpy as np

from constants import IK_DT_S
from robot_arm import animate_q, viser_animate_q, JointPose

SESSION_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")


def _session_file(name: str) -> str:
    return f"{name}_dt-{IK_DT_S}s_{SESSION_TIMESTAMP}.csv"


Q_FRAMES_FILE = _session_file("q_frames")
TARGETS_XYZ_FILE = _session_file("targets_xyz")
TARGETS_POSE_FILE = _session_file("targets_pose")


def record_q_frames(q_frames: np.ndarray):
    n_active = q_frames.shape[-1]
    header = [f"q{i}_rad" for i in range(n_active)]

    file_exists = os.path.exists(Q_FRAMES_FILE)
    with open(Q_FRAMES_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(header)
        writer.writerows(q_frames)


def record_targets(targets_xyz: list):
    xyz_pts, pose_rows = [], []

    for t in targets_xyz:
        if isinstance(t, JointPose):
            pose_rows.append(t.q_active)
        else:
            xyz_pts.append(np.asarray(t, dtype=float).reshape(3))

    if xyz_pts:
        pts = np.asarray(xyz_pts)
        with open(TARGETS_XYZ_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "z"])
            writer.writerows(pts)

    if pose_rows:
        n = len(pose_rows[0])
        with open(TARGETS_POSE_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([f"q{i}_rad" for i in range(n)])
            writer.writerows(pose_rows)


def _load_targets(xyz_file: str | None, pose_file: str | None) -> list | None:
    targets = []

    if xyz_file and os.path.exists(xyz_file):
        pts = np.loadtxt(xyz_file, delimiter=",", skiprows=1)
        if pts.ndim == 1:
            pts = pts.reshape(1, -1)
        targets.extend(list(pts))

    if pose_file and os.path.exists(pose_file):
        rows = np.loadtxt(pose_file, delimiter=",", skiprows=1)
        if rows.ndim == 1:
            rows = rows.reshape(1, -1)
        targets.extend([JointPose(q_active=row) for row in rows])

    return targets or None


def playback(
    urdf_base_link: str,
    urdf_path: str,
    q_frames_file: str,
    xyz_file: str | None = None,
    pose_file: str | None = None,
):
    q_frames = np.loadtxt(q_frames_file, delimiter=",", skiprows=1)
    targets = _load_targets(xyz_file, pose_file)
    animate_q(urdf_base_link, urdf_path, q_frames, targets_xyz=targets)


def playback_viser(
    urdf_base_link: str,
    urdf_path: str,
    q_frames_file: str,
    xyz_file: str | None = None,
    pose_file: str | None = None,
    port: int = 8080,
):
    q_frames = np.loadtxt(q_frames_file, delimiter=",", skiprows=1)
    targets = _load_targets(xyz_file, pose_file)
    viser_animate_q(
        urdf_base_link, urdf_path, q_frames, targets_xyz=targets, port=port
    )
