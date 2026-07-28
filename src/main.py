
'''
July 16 2026 
Olivia Cai

there are four object classes:
Puzzle
Camera
Gripper
RobotMessage

Classes for Camera, Gripper, RobotMessage exist as API wrappers, so that you can easily change the type of device later

Class for Puzzle exists so that you can solve different kinds of puzzles

'''
import json
import os
from datetime import datetime

import cv2
import robot_control
from puzzle import Puzzle
from camera_api import Camera

from calibrate_static import calibrate
import media.puzzles.wiggly.puzzle_solver as puzzle_solver
from const import *

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

import json

from rtde_receive import RTDEReceiveInterface
from configs.robot import ROBOT_IP

from robot_message_send import robot_message_send



def plot_moves(moves, config_path=CONFIG_PATH, scatter_area_px=SCATTER_AREA_PX,
                assembly_offset_m=ASSEMBLY_OFFSET_M, puzzle_size_m=PUZZLE_SOLVED_SIZE_M):
    '''
    this plot function is for debugging/visualization

    visualize each piece's move: blue circle at its current centroid, green
    circle at its target (solved) centroid, a straight arrow for the
    translation, a curved arrow for the rotation_delta_rad about the
    current centroid, and shaded blue/green polygons for the piece's actual
    detected/solved contour (current_contour/target_contour, see
    puzzle_solver.match_and_align).

    the camera only calibrates the scatter region -- target_centroid is fit
    into a footprint sized from puzzle_size_m (const.PUZZLE_SOLVED_SIZE_M),
    anchored at scatter_area_px's own origin, by
    puzzle_solver.puzzle_size_m_to_area_px (see build_solution_key) --
    NOT scatter_area_px's own (generally much bigger) size, since the
    physical assembly area is the puzzle's real footprint, just moved by
    const.ASSEMBLY_OFFSET_M (robot-frame meters). This plot has no direct
    pixel calibration for the assembly area, so it previews it by computing
    that same footprint and shifting it by the offset (converted to pixels
    via config.json's pixel_to_robot_affine) -- a visualization
    approximation, not a second camera-calibrated frame.
    '''
    _, ax = plt.subplots(figsize=(10, 8))

    sx0, sy0, sx1, sy1 = scatter_area_px
    offset_px = (0.0, 0.0)
    assembly_area_px = scatter_area_px  # fallback if calibration/puzzle size unavailable
    if os.path.exists(config_path):
        with open(config_path) as f:
            config = json.load(f)
        robot_w, robot_h = config["output_size"]
        ax.add_patch(Rectangle((0, 0), robot_w, robot_h, fill=False,
                                edgecolor="black", linestyle=":", lw=1, label="robot space"))

        if "pixel_to_robot_affine" in config:
            affine = np.array(config["pixel_to_robot_affine"], dtype=np.float64)
            offset_px = robot_control.robot_delta_to_pixel_delta(*assembly_offset_m, affine)
            if puzzle_size_m is not None:
                assembly_area_px = puzzle_solver.puzzle_size_m_to_area_px(puzzle_size_m, (sx0, sy0), affine)
        else:
            print(f"plot_moves: no pixel_to_robot_affine in {config_path}, "
                  "assembly area preview will overlap the scatter area")
    else:
        print(f"plot_moves: no calibration at {config_path}, skipping robot space bounding box")

    dpx, dpy = offset_px
    ax.add_patch(Rectangle((sx0, sy0), sx1 - sx0, sy1 - sy0, fill=False,
                            edgecolor="blue", linestyle="--", lw=1.5, label="scatter area"))
    ax0, ay0, ax1, ay1 = assembly_area_px
    ax.add_patch(Rectangle((ax0 + dpx, ay0 + dpy), ax1 - ax0, ay1 - ay0, fill=False,
                            edgecolor="green", linestyle="--", lw=1.5, label="assembly area (preview)"))

    for move in moves:
        cx, cy = move["current_centroid"]
        tx, ty = move["target_centroid"][0] + dpx, move["target_centroid"][1] + dpy
        rotation = move["rotation_delta_rad"]
        radius = 0.05 * np.sqrt(move["area"])

        # shaded piece outlines, in absolute (centroid-added-back) coords
        if move.get("current_contour"):
            ax.add_patch(Polygon(move["current_contour"], closed=True, facecolor="blue",
                                  edgecolor="blue", alpha=0.25, lw=1, zorder=1))
        if move.get("target_contour"):
            target_contour_preview = [[x + dpx, y + dpy] for x, y in move["target_contour"]]
            ax.add_patch(Polygon(target_contour_preview, closed=True, facecolor="green",
                                  edgecolor="green", alpha=0.25, lw=1, zorder=1))

        ax.plot(cx, cy, "o", color="blue", markersize=10, zorder=3, label="current" if move is moves[0] else None)
        ax.plot(tx, ty, "o", color="green", markersize=10, zorder=3, label="target" if move is moves[0] else None)
        ax.annotate(f"id {move['id']}", (cx, cy), textcoords="offset points", xytext=(8, 8))

        # translation: straight arrow from current -> target centroid (preview position)
        ax.annotate(
            "", xy=(tx, ty), xytext=(cx, cy),
            arrowprops=dict(arrowstyle="->", color="black", lw=2),
        )

        # rotation: curved arrow swept by rotation_delta_rad around the current centroid
        theta = np.linspace(0, rotation, 50)
        arc_x = cx + radius * np.cos(theta)
        arc_y = cy + radius * np.sin(theta)
        ax.plot(arc_x, arc_y, color="red", lw=2)
        ax.annotate(
            "", xy=(arc_x[-1], arc_y[-1]), xytext=(arc_x[-2], arc_y[-2]),
            arrowprops=dict(arrowstyle="->", color="red", lw=2),
        )

        # label the rotation amount in degrees at the arc's midpoint, offset
        # slightly outward from the centroid so it doesn't overlap the arc itself
        mid_theta = rotation / 2
        label_radius = radius * 1.4
        label_x = cx + label_radius * np.cos(mid_theta)
        label_y = cy + label_radius * np.sin(mid_theta)
        ax.annotate(
            f"{np.degrees(rotation):.1f}°",
            (label_x, label_y),
            color="red", fontsize=9, fontweight="bold",
            ha="center", va="center", zorder=4,
        )

    ax.set_aspect("equal")
    ax.invert_yaxis()  # pixel coords: y grows downward
    ax.set_title("Puzzle piece moves (blue = current, green = target, orange = rotation)")
    ax.set_xlabel("x (px)")
    ax.set_ylabel("y (px)")
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


def main():
    '''
    Hello programmer! This main function does the following:
    1. captures current image
    1. calibrates the image to robot space transform matrix if RECALIBRATION=True
    2. identifies the solution to PUZZLE_PATH and saves this solution as f"solution_key_{PUZZLE_PATH}.json" 
    3. given the current image of the pieces, identify the pieces and match them to the solution key
    4. execute the robot moves to assemble the puzzle

    Things I am Worried About:
    1. noise in the image
    2. calibration error (so, add a bit of margin for each puzzle piece)
    3. singularities with rotating the robot gripper
    
    Architecture:
    - Use classes for all objects that can have multiple kinds of api or type. For instance, can have jigsaw or wavy piece puzzles, so class Puzzle can have an id config where it executes certain functions for a certain type of puzzle
    - the following groups should be black boxes to each other: {Puzzle, Gripper, Robot}
    
    '''
    rtde_r=None
    if not SIMULATION:
        rtde_r = RTDEReceiveInterface(ROBOT_IP)
        robot_control.go_to_pose_camera_capture()
        robot_control.wait_until_robot_stopped(rtde_r)
        

    # first create a puzzle object. optional args but you can just set the values in const.py and then run this prog (puzzletype: str = PUZZLE_TYPE, puzzlepath: str = "media/puzzles/puzzle.png", recalibrate: int = RECALIBRATE, simulated: int = SIMULATION) -> Puzzle
    puzzle = Puzzle()
    cam = Camera()
    if CAPTURE_FRESH:
        cam.capture(image_path=CAPTURE_PATH)
    raw_image = cv2.imread(CAPTURE_PATH)


    if raw_image is None:
        print(f"No capture found at {CAPTURE_PATH}; skipping black/white preview")
    else:
        processed_image = puzzle_solver.clean(raw_image)
        os.makedirs("media/captures_processed", exist_ok=True)
        cv2.imwrite(f"media/captures_processed/{os.path.basename(CAPTURE_PATH)}", processed_image)

    if RECALIBRATE:
        puzzle.config = calibrate() # after this step, calibration config.json exists else prog shld panic

    print(f"loaded config: {CONFIG_PATH}")

    puzzle.build_answerkey()
    print(f"solution key built: {len(puzzle.get_solution_pieces())} pieces")

    moves = puzzle.solve_current()
    # print("moves")
    # print(moves)

    if moves is not None:
        print(f"computed {len(moves)} piece moves -> configs/current_pieces.json")
        plot_moves(moves)
        robot_control.execute_moves(moves, simulated=SIMULATION, rtde_r=rtde_r)
        if not SIMULATION:
            robot_control.go_to_pose_camera_capture() #job is all done 
            robot_control.wait_until_robot_stopped(rtde_r)


        


if __name__ == "__main__":
    main()
    # import cv2
    # from const import CAPTURE_PATH

    # image = cv2.imread(CAPTURE_PATH)

    # # replace with the actual piece ids you observed landing rotated wrong
    # flipped_ids = [0, 5, 9]

    # puzzle_solver.debug_check_flipped_pieces(image, flipped_ids)

    