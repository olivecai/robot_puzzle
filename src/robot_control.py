'''
turns the moves computed by puzzle_solver.process_frame() (translation/rotation
per piece, in rectified-image pixel coordinates) into URScript pick/place
sequences and runs them via the Gripper/send_urscript black boxes.

pixel -> robot-frame XY now comes from the affine matrix fitted in
calibrate_static.py (configs/config.json's "pixel_to_robot_affine"), not
from hand-measured table geometry. See calibrate_static.py for how that
matrix is produced.

DRY_RUN prints each generated command instead of sending it -- keep this True
until the moves have been checked against the real table.
'''

import json
import os
import sys

import numpy as np

from configs.robot import (
    PICK_Z_M,
    PLACE_Z_M,
    SAFE_Z_M,
    TOOL_ORIENTATION_RV,
)
from const import CONFIG_PATH
from gripper_api import Gripper
from send_UR_msg import send_urscript

DRY_RUN = True


def pixel_to_robot_xy(px, py, affine):
    '''
    map a rectified-image pixel to robot base-frame XY (m), using the
    affine transform fitted from robot-touch correspondences
    (configs/config.json's "pixel_to_robot_affine", produced by
    calibrate_static.py's collect_robot_correspondences/fit_pixel_to_robot_transform).
    '''
    x, y = affine @ np.array([px, py, 1.0])
    return float(x), float(y)


def _pose(x, y, z):
    rx, ry, rz = TOOL_ORIENTATION_RV
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def build_pick_place_script(move, affine):
    '''
    URScript movel commands for one piece: travel above it, descend to grip
    height, lift, travel above its target, descend to release height, lift.

    rotation_delta_rad isn't applied to the place pose yet -- TOOL_ORIENTATION_RV
    is a placeholder until the tool's actual rotation axis convention is known.
    '''
    pick_x, pick_y = pixel_to_robot_xy(*move["current_centroid"], affine)
    place_x, place_y = pixel_to_robot_xy(*move["target_centroid"], affine)

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


def execute_move(move, affine, gripper, dry_run=DRY_RUN):
    '''run one piece's full pick -> place sequence.'''
    script = build_pick_place_script(move, affine)

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
        config = json.load(f)

    if "pixel_to_robot_affine" not in config:
        raise KeyError(
            f"{config_path} has no 'pixel_to_robot_affine' -- re-run calibrate_static.calibrate() "
            "(it now fits this matrix as part of calibration)."
        )
    affine = np.array(config["pixel_to_robot_affine"], dtype=np.float64)

    gripper = Gripper(simulated=simulated or dry_run)

    for move in moves:
        print(f"--- piece {move['id']} ---")
        execute_move(move, affine, gripper, dry_run=dry_run)