'''
turns the moves computed by puzzle_solver.process_frame() (translation/rotation
per piece, in rectified-image pixel coordinates) into URScript pick/place
sequences and runs them via the Gripper black boxes.

pixel -> robot-frame XY now comes from the affine matrix fitted in
calibrate_static.py (configs/config.json's "pixel_to_robot_affine"), not
from hand-measured table geometry. See calibrate_static.py for how that
matrix is produced.
'''

import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from const import *

import numpy as np

import matplotlib.pyplot as plt

from configs.robot import (
    PICK_Z_M,
    PLACE_Z_M,
    SAFE_Z_M,
    TOOL_ORIENTATION_RV,
)
from const import CONFIG_PATH, SIMULATION, ROBOT_MESSAGE_TYPE
from gripper_api import Gripper
from robot_message_send import robot_message_send

def pixel_to_robot_xy(px, py, affine):
    '''
    map a rectified-image pixel to robot base-frame XY (m), using the
    affine transform fitted from robot-touch correspondences
    (configs/config.json's "pixel_to_robot_affine", produced by
    calibrate_static.py's collect_robot_correspondences/fit_pixel_to_robot_transform).
    '''
    x, y = affine @ np.array([px, py, 1.0])
    return float(x), float(y)


def robot_delta_to_pixel_delta(dx_m, dy_m, affine):
    '''
    inverse of pixel_to_robot_xy, for a translation rather than a point:
    given a robot base-frame displacement (m), return the equivalent
    displacement in rectified pixels. Only the affine's 2x2 linear
    submatrix matters for a delta (its constant offset column only applies
    to absolute points) -- used by main.plot_moves to preview
    const.ASSEMBLY_OFFSET_M in the same pixel frame as the rest of the
    debug plot.
    '''
    linear = affine[:, :2]
    dpx, dpy = np.linalg.inv(linear) @ np.array([dx_m, dy_m])
    return float(dpx), float(dpy)


def _pose(x, y, z):
    rx, ry, rz = TOOL_ORIENTATION_RV
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def build_pick_place_script(move, affine):
    '''
    
    '''
    if ROBOT_MESSAGE_TYPE == URSCRIPT:
        print("Robot Message Generation...")
        '''
        URScript movel commands for one piece: travel above it, descend to grip
        height, lift, travel above its target, descend to release height, lift.

        rotation_delta_rad isn't applied to the place pose yet -- TOOL_ORIENTATION_RV
        is a placeholder until the tool's actual rotation axis convention is known.
        '''
        pick_x, pick_y = pixel_to_robot_xy(*move["current_centroid"], affine)
        place_x, place_y = pixel_to_robot_xy(*move["target_centroid"], affine)
        place_x += ASSEMBLY_OFFSET_M[0]
        place_y += ASSEMBLY_OFFSET_M[1]

        pick_travel = _pose(pick_x, pick_y, SAFE_Z_M)
        pick_grip = _pose(pick_x, pick_y, PICK_Z_M)
        place_travel = _pose(place_x, place_y, SAFE_Z_M)
        place_release = _pose(place_x, place_y, PLACE_Z_M)

        return {
            "approach_pick": f"movel({pick_travel}, a=0.2, v=0.01)",
            "descend_pick": f"movel({pick_grip}, a=0.2, v=0.2)",
            "lift_after_pick": f"movel({pick_travel}, a=0.2, v=0.01)",
            "approach_place": f"movel({place_travel}, a=0.2, v=0.01)",
            "descend_place": f"movel({place_release}, a=0.2, v=0.01)",
            "lift_after_place": f"movel({place_travel}, a=0.2, v=0.01)",
        }

        

    else: 
        print("Set ROBOT_MESSAGE_TYPE to be a valid robot message type in const.py")
        return None


def execute_move(move, affine, gripper, simulated):
    '''run one piece's full pick -> place sequence.'''
    script = build_pick_place_script(move, affine)

    def run(command):
        if simulated:
            print("LOGGING robot_control run")
            with open("DRYRUN.txt", mode="a") as f:
                f.write(f"LOGGING robot_control.py : [SIMULATION] {command}\n")
            
        else:
            print("Sending Robot Message...")
            robot_message_send(command=command)

    run(script["approach_pick"])
    run(script["descend_pick"])
    gripper.on()
    run(script["lift_after_pick"])
    run(script["approach_place"])
    run(script["descend_place"])
    gripper.off()
    run(script["lift_after_place"])
    




def execute_moves(moves, config_path=CONFIG_PATH, simulated=SIMULATION):
    '''turn each matched piece move into a pick/place sequence and run (or print) it.'''
    with open(config_path) as f:
        config = json.load(f)

    if "pixel_to_robot_affine" not in config:
        raise KeyError(
            f"{config_path} has no 'pixel_to_robot_affine' -- re-run calibrate_static.calibrate() "
            "(it now fits this matrix as part of calibration)."
        )
    affine = np.array(config["pixel_to_robot_affine"], dtype=np.float64)

    gripper = Gripper(simulated=simulated)

    for move in moves:
        # print(f"--- piece {move['id']} ---")
        execute_move(move, affine, gripper, simulated)
        msg = "popup('continue', title='Operator Prompt', blocking=True)"
        robot_message_send(msg)
