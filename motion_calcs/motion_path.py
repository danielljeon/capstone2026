import numpy as np

from robot_arm import SegmentPlan

# Example segment plans.
horizontal_punch_segment = SegmentPlan(
    mode="fixed_vector",
    vector=[0, 1, 0],
    orientation_mode="Z",
    target_orientation=[0, 1, 0],
)
vertical_rslide_segment = SegmentPlan(
    mode="fixed_vector",
    vector=[-1, 0, 0],
    orientation_mode="Z",
    target_orientation=[0, 0, 1],
)
vertical_down_segment = SegmentPlan(
    mode="fixed_vector",
    vector=[0, 0, -1],
    orientation_mode="Z",
    target_orientation=[0, 0, 1],
)
vertical_up_segment = SegmentPlan(
    mode="fixed_vector",
    vector=[0, 0, 1],
    orientation_mode="Z",
    target_orientation=[0, 0, 1],
)

# Targets in meters.
targets = np.array(
    [
        [-0.28, 0.3, 0.15],
        [-0.25, 0.3, 0.15],
        [-0.25, 0.3, 0.0],
        [-0.25, 0.3, 0.15],
    ],
    dtype=float,
)

# One plan per segment between targets (len(targets)-1).
plans = [
    # SegmentPlan(mode="free"),
    vertical_rslide_segment,
    vertical_down_segment,
    vertical_up_segment,
]
