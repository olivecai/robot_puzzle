# fro const values properly export must have CWD in robot_puzzle

### DO NOT edit this section:
# robot message types:
DIGITALOUT_URSCRIPT="digitalout_urscript"
# gripper types:
URSCRIPT="urscript"
# puzzle types:
WIGGLY="wiggly"
# camera types:
DEPTHAI="depthai"
#############


### EDIT this section if you need to run in simulation, capture a new frame with the camera, or recalibrate
## initial set up, have SIMULATION==0, CAPTURE_FRESH==1, RECALIBRATE==1.
## once initial set up is finished and calibration done: SIMULATION==0, CAPTURE_FRESH==01, RECALIBRATE==0
SIMULATION = 1
CAPTURE_FRESH = 0 # if 1: take a new picture with camera. if 0: do not take fresh picture, just search for a photo at CAPTURE_PATH and throw error if does not exist.
RECALIBRATE= 1 # CAUTION recalibrating requires careful measuring with physically moving the arm around and recording the coordinates etc!!


## likely does not need to be edited but if a different camera/puzzle/gripper/message type is needed, you can firstly implement it and then add it here:
PUZZLE_TYPE = WIGGLY # currently jigsaw isnt implemented
ROBOT_MESSAGE_TYPE = URSCRIPT 
GRIPPER_TYPE = DIGITALOUT_URSCRIPT
CAMERA_TYPE = DEPTHAI

PUZZLE_PATH = "media/puzzles/wiggly/Puzzle_12_gap50.png" # png path used for solution key generation (answer key png)
CAPTURE_PATH = "media/captures_raw/photo.png" # png path that the live camera capture saves to (the raw current scrambled real live puzzle)
CONFIG_PATH = "configs/config.json" # json path where configurations are stored ()
CURRENT_PIECES_PATH = "configs/current_pieces.json" # json path where current piece information is saved to (like PCA, moments of each detected piece)
SOLUTION_KEY_PATH = "configs/solution_key.json" # json path of 

SCREEN_WIDTH=2880//4
SCREEN_HEIGHT=1800//4

# robot points recorded by getting the TCP position reported over the UR RTDE; 
# you can easily print these by running `python configs/calibration/scrape_calib_pnts.py`. 
# simply copy and paste them below:`
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

PUZZLE_SOLVED_SIZE_M = (0.3, 0.23) # the 12 piece wiggly puzzle is 0.27 x 0.2 but if generating solution with 50 px gap between pieces, add a few mm to each puzzle dimension

# where to assemble the actual puzzle; offset in real world robot translation applied to the pixel_to_robot_affine conversion in robot_control.py
ASSEMBLY_OFFSET_M = (-0.30, -0.1)

