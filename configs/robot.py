ROBOT_IP = "192.168.1.100"  # your UR's IP
PORT = 30002

# --- physical calibration: rectified table-image pixels -> robot base frame ---
# TODO: replace with real measurements from the physical setup
TABLE_ORIGIN_XY_M = (1000.0, 1000.0)     # robot base-frame XY (m) at rectified image pixel (0, 0) (note that 0,0 is top left corner, +x is rightward and +y is downward)
TABLE_WIDTH_MM = 500.0             # physical width of the calibrated table rectangle
TABLE_HEIGHT_MM = 350.0            # physical height of the calibrated table rectangle
TABLE_ROTATION_RAD = 0.0           # rotation from image axes to robot base axes

# --- TCP heights (m), base frame ---
# TODO: measure against the real table + suction cup length
PICK_Z_M = 0.05     # height to touch/grip a piece with the suction cup
PLACE_Z_M = 0.05    # usually the same as PICK_Z_M
SAFE_Z_M = 0.15      # travel height clear of all pieces

# --- tool orientation (axis-angle rx, ry, rz) for the cup pointing straight down ---
# TODO: verify against your actual TCP/mount -- this is a placeholder, not a
# measured value, and getting it wrong will point the suction cup sideways
TOOL_ORIENTATION_RV = (0.0, 3.1416, 0.0)

# --- Schmalz Cobotpump suction control ---
VACUUM_DIGITAL_OUT = 0  # TODO: which digital output toggles the vacuum