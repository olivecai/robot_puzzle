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
import cv2

from rtde_receive import RTDEReceiveInterface

import time

import matplotlib.pyplot as plt

from configs.robot import (
    PICK_Z_M,
    PLACE_Z_M,
    SAFE_Z_M,
    TOOL_ORIENTATION_RV,
    JOINT_LIMITS_RAD
)

from const import CONFIG_PATH, SIMULATION, ROBOT_MESSAGE_TYPE
from gripper_api import Gripper
from robot_message_send import robot_message_send


def find_safe_wrist3(current_q, rotation_delta_rad, joint_limits=JOINT_LIMITS_RAD,
                      max_wrist_travel_rad=np.pi):
    '''
    For a pure in-plane (world-Z) rotation with the tool pointing straight
    down, only wrist3 needs to change -- base/shoulder/elbow/wrist1/wrist2
    stay at their current values (position and tilt are unaffected by a
    roll-only change). Tries rotation_delta_rad and its +-2pi equivalents,
    picks whichever keeps wrist3 in-limits with the smallest travel.
    '''
    base, shoulder, elbow, wrist1, wrist2, wrist3 = current_q
    lo, hi = joint_limits[5]

    candidates = [-rotation_delta_rad,
              -rotation_delta_rad + 2*np.pi,
              -rotation_delta_rad - 2*np.pi]

    best_wrist3 = None
    best_travel = float("inf")

    for delta in candidates:
        # confirm sign convention against your robot -- may need `wrist3 - delta`
        new_wrist3 = wrist3 + delta
        if not (lo <= new_wrist3 <= hi):
            continue
        travel = abs(delta)
        if travel < best_travel:
            best_wrist3 = new_wrist3
            best_travel = travel

    if best_wrist3 is None:
        return None

    if best_travel > max_wrist_travel_rad:
        print(f"WARNING: {np.degrees(best_travel):.1f} deg of wrist3 travel required "
              f"-- check for an unreasonable rotation.")

    new_q = [base, shoulder, elbow, wrist1, wrist2, best_wrist3]
    return new_q


def compose_rotation_rv(base_rv, delta_theta_rad):
    '''
    Apply an additional rotation of delta_theta_rad about the world Z axis
    on top of the base tool orientation (rotation vector form). This is
    what makes the gripper spin the piece in-plane while keeping the same
    downward-facing tool orientation.
    '''
    R0, _ = cv2.Rodrigues(np.array(base_rv, dtype=np.float64))
    Rz, _ = cv2.Rodrigues(np.array([0.0, 0.0, delta_theta_rad], dtype=np.float64))
    R_new = Rz @ R0
    rv_new, _ = cv2.Rodrigues(R_new)
    return rv_new.flatten()


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


def _pose(x, y, z, rv=None):
    rx, ry, rz = rv if rv is not None else TOOL_ORIENTATION_RV
    return f"p[{x:.4f}, {y:.4f}, {z:.4f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"

def build_pick_place_script(move, affine, rtde_r):
    pick_x, pick_y = pixel_to_robot_xy(*move["current_centroid"], affine)
    place_x, place_y = pixel_to_robot_xy(*move["target_centroid"], affine)
    place_x += ASSEMBLY_OFFSET_M[0]
    place_y += ASSEMBLY_OFFSET_M[1]

    rotation_delta_rad = move["rotation_delta_rad"]

    # rough pre-flight sanity check only -- NOT the value actually sent.
    # Uses whatever joints we're at right now, just to catch a wildly
    # unreachable rotation early; the real baseline is read live below.
    current_q = rtde_r.getActualQ()
    preflight_q = find_safe_wrist3(current_q, rotation_delta_rad)
    if preflight_q is None:
        raise RuntimeError(
            f"piece {move['id']}: rotation_delta_rad={rotation_delta_rad:.3f} "
            f"looks unreachable even as a rough pre-check -- needs manual review."
        )

    pick_travel = _pose(pick_x, pick_y, SAFE_Z_M)      # unrotated -- no Rodrigues needed
    pick_grip = _pose(pick_x, pick_y, PICK_Z_M)
    place_travel = _pose(place_x, place_y, SAFE_Z_M)   # unrotated -- arrives here, THEN rotates

    lo, hi = JOINT_LIMITS_RAD[5]
    margin = np.radians(10)

    return {
        "approach_pick": f"movel({pick_travel}, a=0.2, v=0.04)",
        "descend_pick": f"movel({pick_grip}, a=0.2, v=0.04)",
        "lift_after_pick": f"movel({pick_travel}, a=0.2, v=0.04)",
        "approach_place": f"movel({place_travel}, a=0.2, v=0.04)",
        "rotate_at_place": f"""
        local q_now = get_actual_joint_positions()
        local target_w3 = q_now[5] + ({rotation_delta_rad})
        if target_w3 > {lo + margin} and target_w3 < {hi - margin}:
            q_now[5] = target_w3
        elif q_now[5] + ({rotation_delta_rad} + 2*3.14159265) < {hi - margin}:
            q_now[5] = q_now[5] + ({rotation_delta_rad} + 2*3.14159265)
        elif q_now[5] + ({rotation_delta_rad} - 2*3.14159265) > {lo + margin}:
            q_now[5] = q_now[5] + ({rotation_delta_rad} - 2*3.14159265)
        end
        movej(q_now, a=0.3, v=0.3)
        """,
                "descend_place": f"""
        local p_now = get_actual_tcp_pose()
        p_now[2] = {PLACE_Z_M}
        movel(p_now, a=0.2, v=0.04)
        """,
                "lift_after_place": f"""
        local p_now = get_actual_tcp_pose()
        p_now[2] = {SAFE_Z_M}
        movel(p_now, a=0.2, v=0.04)
        """,
            }


def execute_move(move, affine, gripper, simulated, rtde_r):
    script = build_pick_place_script(move, affine, rtde_r)

    full_script = f"""
def pick_place_sequence():
    {script["approach_pick"]}
    {script["descend_pick"]}
    set_tool_digital_out(0, False)
    set_tool_digital_out(1, True)
    {script["lift_after_pick"]}
    {script["approach_place"]}
    {script["rotate_at_place"]}
    {script["descend_place"]}
    set_tool_digital_out(1, False)
    set_tool_digital_out(0, True)
    {script["lift_after_place"]}
end
pick_place_sequence()
"""
    with open("ROBOTMOVES.txt", mode="a") as f:
        f.write(f"LOGGING robot_control.py : [SIMULATION]\n{full_script}\n")
    if not simulated:
        print("Sending Robot Message...")
        robot_message_send(command=full_script)


def execute_moves(moves, rtde_r, config_path=CONFIG_PATH, simulated=SIMULATION):
    with open(config_path) as f:
        config = json.load(f)

    if "pixel_to_robot_affine" not in config:
        raise KeyError(
            f"{config_path} has no 'pixel_to_robot_affine' -- re-run calibrate_static.calibrate() "
        )
    affine = np.array(config["pixel_to_robot_affine"], dtype=np.float64)

    gripper = Gripper(simulated=simulated)


    for move in moves:
        if not simulated:
            go_to_pose_centroid()
            wait_until_robot_stopped(rtde_r)
        execute_move(move, affine, gripper, simulated, rtde_r)
        if not simulated:
            wait_until_robot_stopped(rtde_r)


def go_to_pose_camera_capture():
    with open("configs/camera_capture_joint_pose.json") as f:
        data = json.load(f)

    joints = data["joints"]

    script = f"""
def go_to_saved_pose():
    movej([{joints[0]}, {joints[1]}, {joints[2]}, {joints[3]}, {joints[4]}, {joints[5]}], a=0.2, v=0.4)
end
go_to_saved_pose()
"""
    robot_message_send(script)

def go_to_pose_centroid():
    with open("configs/calibration/centroid.json") as f:
        data = json.load(f)

    joints = data["joints"]

    script = f"""
def go_to_saved_pose():
    movej([{joints[0]}, {joints[1]}, {joints[2]}, {joints[3]}, {joints[4]}, {joints[5]}], a=0.2, v=0.8)
end
go_to_saved_pose()
"""
    robot_message_send(script)

def wait_until_robot_stopped(rtde_r, speed_threshold=0.001, stable_duration=0.3,
                        grace_period=0.3, timeout=100):
    time.sleep(grace_period)
    start_time = time.time()
    stable_since = None
    while time.time() - start_time < timeout:
        speed = rtde_r.getActualTCPSpeed()
        linear_speed = np.linalg.norm(speed[:3])
        angular_speed = np.linalg.norm(speed[3:6])
        combined_speed = max(linear_speed, angular_speed)  # or check both independently

        if combined_speed < speed_threshold:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= stable_duration:
                return True
        else:
            stable_since = None
        time.sleep(0.02)
    print("wait_until_robot_stopped: timed out")
    return False

