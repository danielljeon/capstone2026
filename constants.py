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

# IK and URDF driven constants.

# Environment variables load.
load_dotenv()  # Load variables from .env.
URDF_BASE_LINK = os.getenv("URDF_BASE_LINK", "base")
URDF_PATH = os.getenv("URDF_PATH", "./urdf/robot.urdf")

"""IK calculation and execution frame time step (seconds)."""
IK_DT_S = 0.02

"""URDF zero pose EE position and rotation matrix."""
ZERO_POSE_EE_POS, ZERO_POSE_EE_R = fk_ee(
    URDF_BASE_LINK, URDF_PATH, [0, 0, 0, 0, 0, 0]
)

"""Startup safer zero pose."""
START_POSE = JointPose([0, -0.85, 0, 0, -0.85, 0])

"""Nice camera and z-fold pose."""
OPTIMAL_POSE = JointPose([0, 0, 0.17, 3.32, 0, 2.27])
