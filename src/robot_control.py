'''
turns the moves computed by puzzle_solver.process_frame() (translation/rotation
per piece, in rectified-image pixel coordinates) into URScript pick/place
sequences and runs them via the Gripper/send_urscript black boxes.

DRY_RUN prints each generated command instead of sending it -- keep this True
until the moves have been checked against the real table.
'''

import json
import math
import os
import sys

# configs/ lives one level above src/, at the repo root -- put the repo root
# on sys.path so `configs` is importable regardless of cwd or entry point
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from configs.robot import (
    PICK_Z_M,
    PLACE_Z_M,
    SAFE_Z_M,
    TABLE_HEIGHT_MM,
    TABLE_ORIGIN_XY_M,
    TABLE_ROTATION_RAD,
    TABLE_WIDTH_MM,
    TOOL_ORIENTATION_RV,
)
from const import CONFIG_PATH
from gripper_api import Gripper
from send_UR_msg import send_urscript

DRY_RUN = True


def pixel_to_robot_xy(px, py, output_size):
    '''
    map a rectified-image pixel to robot base-frame XY (m), using the
    table's known physical size (configs/robot.py) and the calibrated
    rectangle's pixel size (configs/config.json's output_size).
    '''
    sx = (TABLE_WIDTH_MM / 1000) / output_size[0]
    sy = (TABLE_HEIGHT_MM / 1000) / output_size[1]
    x_local = px * sx
    y_local = py * sy

    c, s = math.cos(TABLE_ROTATION_RAD), math.sin(TABLE_ROTATION_RAD)
    x = c * x_local - s * y_local + TABLE_ORIGIN_XY_M[0]
    y = s * x_local + c * y_local + TABLE_ORIGIN_XY_M[1]
    return x, y


def _pose(x, y, z):
    rx, ry, rz = TOOL_ORIENTATION_RV
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def build_pick_place_script(move, output_size):
    '''
    URScript movel commands for one piece: travel above it, descend to grip
    height, lift, travel above its target, descend to release height, lift.

    rotation_delta_rad isn't applied to the place pose yet -- TOOL_ORIENTATION_RV
    is a placeholder until the tool's actual rotation axis convention is known.
    '''
    pick_x, pick_y = pixel_to_robot_xy(*move["current_centroid"], output_size)
    place_x, place_y = pixel_to_robot_xy(*move["target_centroid"], output_size)

    pick_travel = _pose(pick_x, pick_y, SAFE_Z_M)
    pick_grip = _pose(pick_x, pick_y, PICK_Z_M)
    place_travel = _pose(place_x, place_y, SAFE_Z_M)
    place_release = _pose(place_x, place_y, PLACE_Z_M)

    return {
        "approach_pick": f"movel({pick_travel}, a=1.0, v=0.5)",
        "descend_pick": f"movel({pick_grip}, a=1.0, v=0.2)",
        "lift_after_pick": f"movel({pick_travel}, a=1.0, v=0.5)",
        "approach_place": f"movel({place_travel}, a=1.0, v=0.5)",
        "descend_place": f"movel({place_release}, a=1.0, v=0.2)",
        "lift_after_place": f"movel({place_travel}, a=1.0, v=0.5)",
    }


def execute_move(move, output_size, gripper, dry_run=DRY_RUN):
    '''run one piece's full pick -> place sequence.'''
    script = build_pick_place_script(move, output_size)

    def run(command):
        if dry_run:
            print(f"  [DRY RUN] {command}")
        else:
            send_urscript(command)

    run(script["approach_pick"])
    run(script["descend_pick"])
    gripper.on()
    run(script["lift_after_pick"])
    run(script["approach_place"])
    run(script["descend_place"])
    gripper.off()
    run(script["lift_after_place"])


def execute_moves(moves, config_path=CONFIG_PATH, dry_run=DRY_RUN, simulated=True):
    '''turn each matched piece move into a pick/place sequence and run (or print) it.'''
    with open(config_path) as f:
        output_size = tuple(json.load(f)["output_size"])

    gripper = Gripper(simulated=simulated or dry_run)

    for move in moves:
        print(f"--- piece {move['id']} ---")
        execute_move(move, output_size, gripper, dry_run=dry_run)
