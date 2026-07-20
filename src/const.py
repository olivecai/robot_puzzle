# fro const values properly export must have CWD in robot_puzzle

# dont edit the below
DIGITALOUT_URSCRIPT="digitalout_urscript"
URSCRIPT="urscript"
WIGGLY="wiggly"
#############


# you can edit the below

SIMULATION = 1
RECALIBRATE=0


PUZZLE_TYPE = WIGGLY
ROBOT_MESSAGE_TYPE = URSCRIPT
GRIPPER_TYPE = DIGITALOUT_URSCRIPT

PUZZLE_PATH = f"media/puzzles/{PUZZLE_TYPE}/Puzzle_12.png"
CAPTURE_PATH = "media/captures_processed/Puzzle_24_capture_processed.png"
CONFIG_PATH = "configs/config.json"
CURRENT_PIECES_PATH = "configs/current_pieces.json"
SOLUTION_KEY_PATH = "configs/solution_key.json"

SCREEN_WIDTH=2880//4
SCREEN_HEIGHT=1800//4

# known robot-frame XY (meters) of fixed physical markers on the table,
# used by calibrate_static.py to fit the pixel -> robot affine transform.
# Order matters: during calibration you'll click the corresponding marker
# in the rectified image in this same order. Use at least 3 points, ideally
# 6+ spread across the table (not clustered) for a well-conditioned fit.
CALIBRATION_ROBOT_POINTS = [
    (0.100, 0.100),
    (0.500, 0.100),
    (0.100, 0.500),
    (0.500, 0.500),
    (0.300, 0.300),
    (0.300, 0.550),
]

