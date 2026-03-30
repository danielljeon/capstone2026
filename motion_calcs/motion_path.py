from robot_arm import SegmentPlan, JointPose

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
targets = [
    JointPose([0.52, 0, 2.97, -0.17, 0, 0]),
    [-0.2, 0.3, 0.15],
    [-0.18, 0.3, 0.15],
    [-0.18, 0.3, 0.0],
    [-0.18, 0.3, 0.15],
]

# One plan per segment between targets (len(targets)-1).
plans = [
    # SegmentPlan(mode="free"),
    None,
    vertical_rslide_segment,
    vertical_down_segment,
    vertical_up_segment,
]
