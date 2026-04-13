"""constants.py"""

import os

from dotenv import load_dotenv

from robot_arm import JointPose, fk_ee

# CAN bus.
PWM_NODE_SERVO_INTERFACE = "socketcan"
PWM_NODE_SERVO_CHANNEL = "can0"
PWM_NODE_SERVO_BITRATE = 500_000

# Serial ports.
RSBL120_PORT = "COM6"  # "/dev/cu.usbmodem5ABA0052171"
ST3215_PORT = "COM5"  # "/dev/cu.usbmodem5ABA0050551"

# Run configurations.
ALL_SAFETY_ON = True  # Require manual developer confirm "yes" in terminal.
RECORD_ALL = False  # Record all computed q_active, target_xyz and target_pose.
CAMERA_RECORD = False  # Record view from the RealSense camera.
RUN_VIRTUAL = False  # System wide operation/joint angle tracking mode.
VISER_ANIMATE_ALL = False  # Viser animate all moves.
ANIMATE_ALL = False  # Animate (matplotlib) all moves.
TOOL_LOG = False  # Record all current data for tool movement.
TOOL_LOG_PLOT = False  # Plot each recorded current data for tool movement.

# IK and URDF driven constants.

# Environment variables load.
load_dotenv()  # Load variables from .env.
URDF_BASE_LINK = os.getenv("URDF_BASE_LINK", "base")
URDF_PATH = os.getenv("URDF_PATH", "./urdf/robot.urdf")

"""IK calculation and execution frame time step (seconds)."""
IK_DT_S = 0.01

"""URDF zero pose EE position and rotation matrix."""
ZERO_POSE_EE_POS, ZERO_POSE_EE_R = fk_ee(
    URDF_BASE_LINK, URDF_PATH, [0, 0, 0, 0, 0, 0]
)

"""Startup safer zero pose."""
START_POSE = JointPose([0, -0.85, 0, 0, -0.85, 0])

"""Nice camera and z-fold pose."""
OPTIMAL_POSE = JointPose([0, 0, -0.17, -3.31, 0, 2.27])
