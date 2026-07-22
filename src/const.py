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
RECALIBRATE= 0
CAPTURE_FRESH = 0


PUZZLE_TYPE = WIGGLY
ROBOT_MESSAGE_TYPE = URSCRIPT
GRIPPER_TYPE = DIGITALOUT_URSCRIPT
CAMERA_TYPE = DEPTHAI

PUZZLE_PATH = "media/puzzles/wiggly/Puzzle_12.png" # png path used for solution key generation
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
    (-0.300,-0.516), # top left
    (-0.298,-0.930), # top right
    (-0.601,-0.516), # bottom left
    (-0.601, -0.930), #bottom right
    (-0.450, -0.722) # centroid
]

# the camera only calibrates/sees the scatter region -- SCATTER_AREA_PX is
# that region, in rectified-image pixel coords (the same frame
# current_centroid lives in; config.json's "output_size" is the full-table
# extent this should sit inside of). (x0, y0, x1, y1).
# TODO: measure against your physical table layout once you have a real
# table with a dedicated scatter zone -- for now this matches the full
# current config.json output_size ([8908, 6691]) so synthetic-image test
# runs render at the correct scale. If you regenerate config.json against a
# differently-sized capture, update this to match its new output_size or
# it'll silently go stale again (see plot_moves' size-mismatch symptom).
SCATTER_AREA_PX = (0, 0, 1300, 900)

# real physical size (m) of the fully assembled puzzle -- x along the same
# axis as CALIBRATION_ROBOT_POINTS' first coordinate, y along the second.
# puzzle_solver.build_solution_key uses this (converted to pixels via the
# calibration's real scale) to size the solved layout accurately, instead
# of the coarser SCATTER_AREA_PX-footprint approximation.

PUZZLE_SOLVED_SIZE_M = (0.35, 0.2)

# the assembly area is a *different physical spot* on the table that the
# camera never needs to see -- it's reached purely by this fixed real-world
# translation (robot base-frame meters) applied on top of the normal
# pixel_to_robot_affine conversion, at place time (see robot_control.py).
ASSEMBLY_OFFSET_M = (-0.4, 0.0)

