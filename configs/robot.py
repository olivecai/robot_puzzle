import numpy as np


ROBOT_IP = "10.42.0.167"  # your UR's IP
PORT = 30001 # Primary Client Interface


PICK_Z_M = 0.13500756777487412     # height to touch/grip a piece with the suction cup
PLACE_Z_M = 0.13500756777487412   # usually the same as PICK_Z_M
SAFE_Z_M = 0.170      # travel height clear of all pieces

TOOL_ORIENTATION_RV = (0.0, 3.1416, 0.0)

# joint limits copied from Installation > Safety > Joint Limits
JOINT_LIMITS_DEG = [(-363, 363),
                    (-363, 363),
                    (-363, 363),
                    (-363, 363),
                    (-363, 363),
                    (-363, 363),
                    ]

JOINT_LIMITS_RAD = [
    (np.radians(lo), np.radians(hi)) for lo, hi in JOINT_LIMITS_DEG
]