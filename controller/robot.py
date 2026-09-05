from __future__ import annotations

N_JOINTS = 7

LEFT = [f"left_fr3v2_joint{i}" for i in range(1, 8)]
RIGHT = [f"right_fr3v2_joint{i}" for i in range(1, 8)]
SPINE_JOINT = "franka_spine_vertical_joint"

GRIP_OPEN = 0.8
GRIP_CLOSE = 0.2

ARM_READY = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]

SPINE_H = 0.5
