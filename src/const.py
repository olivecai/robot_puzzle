# fro const values properly export must have CWD in robot_puzzle

### dont edit this section.
# robot message types:
DIGITALOUT_URSCRIPT="digitalout_urscript"
# gripper types:
URSCRIPT="urscript"
# puzzle types:
WIGGLY="wiggly"
# camera types:
DEPTHAI="depthai"
#############


### edit the vars below 

SIMULATION = 0
CAPTURE_FRESH = 1

# DO NOT RECALIBRATE UNLESS STRICTLY MUST
RECALIBRATE= 0

PUZZLE_TYPE = WIGGLY
ROBOT_MESSAGE_TYPE = URSCRIPT
GRIPPER_TYPE = DIGITALOUT_URSCRIPT
CAMERA_TYPE = DEPTHAI

PUZZLE_PATH = "media/puzzles/wiggly/Puzzle_12_gap50.png" # png path used for solution key generation
CAPTURE_PATH = "media/captures_raw/photo.png" # png path that the live camera capture saves to
CONFIG_PATH = "configs/config.json" # json path where configurations are stored
CURRENT_PIECES_PATH = "configs/current_pieces.json" # json path where current piece information is saved to (like PCA, moments of each detected piece)
SOLUTION_KEY_PATH = "configs/solution_key.json" # json path of 

SCREEN_WIDTH=2880//4
SCREEN_HEIGHT=1800//4

# known robot-frame XY (meters) of fixed physical markers on the table,
# used by calibrate_static.py to fit the pixel -> robot affine transform.
# Order matters: during calibration you'll click the corresponding marker
# in the rectified image in this same order. Use at least 3 points, ideally
# 6+ spread across the table (not clustered) for a well-conditioned fit.
CALIBRATION_ROBOT_POINTS = [
    (-0.324515, -0.535946),
    (-0.324433, -0.665751),
    (-0.325629, -0.860529),
    (-0.453511, -0.536058),
    (-0.455554, -0.731123),
    (-0.455542, -0.861135),
    (-0.583241, -0.599751),
    (-0.585706, -0.860013),
]

# area of the scattered puzzle pieces in pixels (you can copy this directly from the x1,y1 coords in config.json)
SCATTER_AREA_PX = (0, 0,1506, 857)

PUZZLE_SOLVED_SIZE_M_ACTUAL = (0.27,0.2) # this isnt used in computation anywhere its just for reference as you toggle PUZZLE_SOLVED_SIZE_M

# the below
PUZZLE_SOLVED_SIZE_M = (0.3, 0.23) # the 12 piece wiggly puzzle is 0.27 x 0.2 but when feeding 

# where to assemble the actual puzzle; offset in real world robot translation applied to the pixel_to_robot_affine conversion in robot_control.py
ASSEMBLY_OFFSET_M = (-0.30, -0.1)

