import json

import cv2
import numpy as np

import robot_control
from const import *

# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def piece_orientation(cnt):
    '''
    determine a jigsaw piece's rotation, in [-pi, pi), from its minimum-area
    bounding rectangle rather than an inertia-tensor PCA axis (as used for
    the "wiggly" puzzle type -- see puzzles/wiggly/puzzle_solver.py's
    principal_angle). Standard jigsaw pieces are near-square with small
    tab/blank perturbations on each side; PCA on the mass distribution can
    be unstable for this shape family -- a tab on one side and a blank on
    the opposite side partially cancel in the inertia tensor, and a
    near-square piece doesn't have the strongly elongated principal axis
    PCA needs to be well-conditioned. minAreaRect instead fits directly to
    the piece's dominant square body, which stays stable regardless of
    where its tabs/blanks happen to be.

    cnt must be centroid-relative (mean-subtracted), matching the
    convention used everywhere else in this module.

    minAreaRect's angle is only defined mod 90 degrees (an axis-aligned
    square looks identical rotated by any multiple of 90). The correct one
    of the 4 candidate quadrant rotations is resolved by picking whichever
    one points the piece's single farthest-from-center point (almost
    always the tip of its most prominent tab) closest to +x. This is an
    arbitrary choice of "zero rotation" for a given piece shape, but a
    *reproducible* one -- which is all a reference_angle zero-point needs
    to be, since rotation_delta_rad is always a difference between two
    calls of this same function (see match_and_align).
    '''
    rect = cv2.minAreaRect(cnt.astype(np.float32))
    base_angle = np.deg2rad(rect[-1])
    candidates = [base_angle + k * (np.pi / 2) for k in range(4)]

    farthest = cnt[np.argmax(np.linalg.norm(cnt, axis=1))]
    farthest_angle = np.arctan2(farthest[1], farthest[0])

    def ang_diff(a, b):
        return abs((a - b + np.pi) % (2 * np.pi) - np.pi)

    best = min(candidates, key=lambda c: ang_diff(farthest_angle, c))
    return (best + np.pi) % (2 * np.pi) - np.pi


def classify_piece_edges(cnt, flat_thresh_ratio=0.06):
    '''
    classify each of a piece's 4 sides -- in minAreaRect box-corner order
    -- as "tab" (bulges outward past the piece's dominant square body),
    "blank" (notches inward), or "flat" (an unmodified outer
    puzzle-boundary edge, no interlock). This is the genuinely
    jigsaw-specific signature: unlike a generic wiggly region, a real
    jigsaw piece's identity is defined by which of its 4 sides interlock
    and how, not just its overall silhouette.

    Every contour point is assigned to whichever side's outward-normal
    direction (from the box center) its own angular position is closest
    to -- a robust partition into 4 "pie slices" that works even for the
    large, round tabs typical of real jigsaw pieces (a linear projection
    onto each side's segment, windowed by position along it, was tried
    first and broke down here: a big tab's points can satisfy a
    neighboring side's window too, without a proximity/angle check to
    keep them separated).

    flat_thresh_ratio: how far (relative to the box's average side length)
    a side's contour must bulge/notch to count as a tab/blank rather than
    noise/flat. Tune up if flat edges are being misread as shallow
    tabs/blanks; down if real tabs/blanks aren't being detected.

    Returns a list of 4 strings; edges[i] is the side between
    cv2.boxPoints(cv2.minAreaRect(cnt))[i] and [(i + 1) % 4].
    '''
    rect = cv2.minAreaRect(cnt.astype(np.float32))
    box_center = np.array(rect[0], dtype=np.float64)
    box = cv2.boxPoints(rect).astype(np.float64)
    side_len = np.mean([np.linalg.norm(box[(i + 1) % 4] - box[i]) for i in range(4)])
    thresh = flat_thresh_ratio * side_len

    def ang_diff(a, b):
        return np.abs((a - b + np.pi) % (2 * np.pi) - np.pi)

    normals = []
    for i in range(4):
        p0, p1 = box[i], box[(i + 1) % 4]
        edge_dir = (p1 - p0) / np.linalg.norm(p1 - p0)
        normal = np.array([-edge_dir[1], edge_dir[0]])
        if np.dot(normal, (p0 + p1) / 2 - box_center) < 0:
            normal = -normal  # make sure it actually points away from the box center
        normals.append(normal)
    normal_angles = np.array([np.arctan2(n[1], n[0]) for n in normals])

    rel_to_center = cnt - box_center
    point_angles = np.arctan2(rel_to_center[:, 1], rel_to_center[:, 0])
    owner = np.argmin([ang_diff(point_angles, na) for na in normal_angles], axis=0)

    edges = []
    for i in range(4):
        pts = cnt[owner == i]
        if len(pts) == 0:
            edges.append("flat")
            continue

        p0 = box[i]
        d = (pts - p0) @ normals[i]
        max_out = d.max()
        max_in = -d.min()

        if max_out > thresh and max_out >= max_in:
            edges.append("tab")
        elif max_in > thresh:
            edges.append("blank")
        else:
            edges.append("flat")

    return edges


def contour_of_mask(mask):
    '''
    return the largest external contour found in a binary mask, as an
    (N, 2) float array. Uses CHAIN_APPROX_NONE (every boundary pixel), not
    CHAIN_APPROX_SIMPLE -- classify_piece_edges needs actual point density
    along flat/straight runs to detect them; CHAIN_APPROX_SIMPLE compresses
    a straight edge down to just its 2 endpoints, which starves flat sides
    of data while curvy tab/blank sides stay dense, and was misreading
    every flat edge as a tab/blank.
    '''
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnt = max(contours, key=cv2.contourArea)
    return cnt.reshape(-1, 2).astype(np.float64)


def warp(image, config_path=CONFIG_PATH):
    '''apply the one-time calibration perspective transform + crop.'''
    with open(config_path) as f:
        config = json.load(f)
    M = np.array(config["M"], dtype=np.float64)
    output_size = tuple(config["output_size"])
    # borderValue=white: any area outside the source image after warping
    # (e.g. source smaller than output_size) must read as background, not
    # black, or it gets misread as a giant extra piece downstream.
    return cv2.warpPerspective(image, M, output_size, borderValue=(255, 255, 255))


# ---------------------------------------------------------------------------
# Live-capture pipeline: pieces are solid black blobs on a white table
# ---------------------------------------------------------------------------

def clean(image, floor=150, ceiling=200, config_path=CONFIG_PATH):
    '''
    crop and edit image for processing --> environment is calibrated once
    initially; there exists a transformation matrix from valid cartesian
    space to image space.

    floor/ceiling are black-point/white-point brightness values: pixels at
    or below floor become black, at or above ceiling become white, and
    values in between are stretched linearly before a final binary
    threshold.
    '''
    cropped = warp(image, config_path)
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

    stretched = np.clip(gray, floor, ceiling).astype(np.float32)
    stretched = (stretched - floor) * (255.0 / (ceiling - floor))

    _, bw = cv2.threshold(stretched.astype(np.uint8), 230, 255, cv2.THRESH_BINARY)
    return bw


def detect_blobs(bw, min_area=200):
    '''
    find each piece blob in a cleaned (binary) image. returns each blob's
    centroid (in full-image coordinates), its contour re-centered on that
    centroid (so shape comparison is position-independent), and its 4-side
    tab/blank/flat signature (see classify_piece_edges).

    table is pure white (255), pieces are pure black (0).
    '''
    piece_mask = np.where(bw == 0, 255, 0).astype(np.uint8)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(piece_mask, connectivity=8)

    blobs = []
    for label in range(1, num_labels):  # label 0 is background
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue

        cx, cy = centroids[label]
        blob_mask = np.where(labels == label, 255, 0).astype(np.uint8)
        cnt = contour_of_mask(blob_mask)
        cnt -= [cx, cy]  # center on centroid for rotation-only comparison

        blobs.append({
            "centroid": [round(float(cx), 2), round(float(cy), 2)],
            "area": area,
            "contour": cnt,
            "edge_types": classify_piece_edges(cnt),
        })

    return blobs


# ---------------------------------------------------------------------------
# Solution-key pipeline: answer key is a line drawing; pieces are the
# regions enclosed by the curves, not filled blobs.
# ---------------------------------------------------------------------------

def extract_regions(image, min_area=5000, line_threshold=200, close_iters=1):
    '''
    segment a line-drawing puzzle image into its enclosed regions (pieces).

    line_threshold: gray value below which a pixel is considered part of a
    boundary curve (curves are dark strokes on a white background).
    close_iters: dilation passes applied to the line mask to bridge small
    gaps in hand-drawn/sketchy curves so every region is fully enclosed.
    Increase if regions are bleeding into each other (check num_labels
    against your expected piece count); decrease if regions are being
    over-split by noise.
    '''
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image

    _, line_mask = cv2.threshold(gray, line_threshold, 255, cv2.THRESH_BINARY_INV)
    if close_iters > 0:
        kernel = np.ones((3, 3), np.uint8)
        line_mask = cv2.dilate(line_mask, kernel, iterations=close_iters)

    region_mask = cv2.bitwise_not(line_mask)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(region_mask, connectivity=4)

    regions = []
    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue  # discard slivers / the outer background region if it leaks out

        cx, cy = centroids[label]
        region_binary = np.where(labels == label, 255, 0).astype(np.uint8)
        cnt = contour_of_mask(region_binary)
        cnt_centered = cnt - [cx, cy]

        regions.append({
            "centroid": [round(float(cx), 2), round(float(cy), 2)],
            "area": area,
            "contour": cnt_centered,
            "edge_types": classify_piece_edges(cnt_centered),
        })

    return regions


def fit_scale_and_offset(src_size, area_px):
    '''
    uniform, aspect-preserving scale + offset that centers a src_size=(w, h)
    frame inside area_px=(x0, y0, x1, y1). returns (scale, (offset_x,
    offset_y)) such that mapped_point = original_point * scale + offset.

    uniform scale (same factor on x and y) is required so that angles --
    piece_orientation/matchShapes -- aren't distorted; an independent x/y
    scale would stretch piece shapes.
    '''
    src_w, src_h = src_size
    x0, y0, x1, y1 = area_px
    area_w, area_h = x1 - x0, y1 - y0

    scale = min(area_w / src_w, area_h / src_h)
    offset_x = x0 + (area_w - src_w * scale) / 2
    offset_y = y0 + (area_h - src_h * scale) / 2
    return scale, (offset_x, offset_y)


def puzzle_size_m_to_area_px(size_m, origin_px, affine):
    '''
    convert the assembled puzzle's real physical size (const.PUZZLE_SOLVED_SIZE_M)
    into a pixel-space area_px box (x0, y0, x1, y1) anchored at origin_px,
    using the calibration's real scale (robot_control.robot_delta_to_pixel_delta)
    -- gives an accurately-scaled fit target instead of the coarser
    SCATTER_AREA_PX-footprint approximation.
    '''
    width_m, height_m = size_m
    ox, oy = origin_px
    wx, wy = robot_control.robot_delta_to_pixel_delta(width_m, 0.0, affine)
    hx, hy = robot_control.robot_delta_to_pixel_delta(0.0, height_m, affine)
    w_px = float(np.hypot(wx, wy))
    h_px = float(np.hypot(hx, hy))
    return (ox, oy, ox + w_px, oy + h_px)


def build_solution_key(image, config_path=CONFIG_PATH, output_path=SOLUTION_KEY_PATH,
                        min_area=5000, apply_calibration=True, area_px=SCATTER_AREA_PX,
                        puzzle_size_m=PUZZLE_SOLVED_SIZE_M):
    '''
    process an answer-key photo into solution_key.json: one entry per
    piece, with its target centroid (where it belongs), reference contour
    (its shape in solved orientation), reference angle (its "solved"
    rotation, used as the zero-point for rotation deltas later, see
    piece_orientation), and its tab/blank/flat edge signature (see
    classify_piece_edges).

    apply_calibration: set False if the answer key was captured/generated
    independently of the robot's calibrated camera frame (e.g. a rendered
    template rather than a live photo of the physical table) and doesn't
    need the perspective warp. If True, it's run through the same warp()
    used on live captures so coordinates line up with robot-frame targets.

    area_px: rectangle (const.SCATTER_AREA_PX), in the same rectified pixel
    frame current_centroid lives in. The solved layout is fit into this
    footprint's origin (top-left corner) -- not a separate assembly
    rectangle -- because the physical assembly area is the *same shape*,
    just moved by const.ASSEMBLY_OFFSET_M (robot-frame meters, applied in
    robot_control.py); the camera never calibrates the assembly area
    directly.

    puzzle_size_m: real physical (width_m, height_m) of the assembled
    puzzle (const.PUZZLE_SOLVED_SIZE_M). When given and config_path has a
    "pixel_to_robot_affine", this replaces area_px's *size* (keeping its
    origin) with an accurately-scaled box via puzzle_size_m_to_area_px --
    otherwise falls back to area_px's own size as a coarser approximation.

    Either way, every target_centroid/reference_contour is rescaled+shifted
    (see fit_scale_and_offset) from the template image's own pixel frame
    into the resulting footprint, so translation = target_centroid -
    current_centroid is computed within one consistent frame instead of
    comparing two unrelated pixel spaces.
    '''
    with open(config_path) as f:
        config = json.load(f)

    frame = warp(image, config_path) if apply_calibration else image
    regions = extract_regions(frame, min_area=min_area)

    if not regions:
        raise ValueError("No enclosed regions found — check line_threshold/close_iters against this image.")

    affine = np.array(config["pixel_to_robot_affine"], dtype=np.float64) if "pixel_to_robot_affine" in config else None

    if puzzle_size_m is not None:
        if affine is not None:
            area_px = puzzle_size_m_to_area_px(puzzle_size_m, area_px[:2], affine)
        else:
            print(f"build_solution_key: no pixel_to_robot_affine in {config_path}, "
                  "falling back to area_px's own size instead of puzzle_size_m")

    src_h, src_w = frame.shape[:2]
    scale, (offset_x, offset_y) = fit_scale_and_offset((src_w, src_h), area_px)

    pieces = []
    for i, region in enumerate(regions):
        cx, cy = region["centroid"]
        pieces.append({
            "id": i,
            "target_centroid": [round(cx * scale + offset_x, 2), round(cy * scale + offset_y, 2)],
            # centroid-relative, so only the scale applies (no offset)
            "reference_contour": (region["contour"] * scale).tolist(),
            "reference_angle": round(float(piece_orientation(region["contour"])), 4),
            "edge_types": region["edge_types"],
            "area": round(region["area"] * scale ** 2, 2),
        })

    with open(output_path, "w") as f:
        json.dump({"pieces": pieces}, f, indent=4)

    return pieces


def load_solution(solution_path=SOLUTION_KEY_PATH):
    with open(solution_path) as f:
        data = json.load(f)

    for p in data["pieces"]:
        p["reference_contour"] = np.array(p["reference_contour"], dtype=np.float64)
    return data["pieces"]


# ---------------------------------------------------------------------------
# Matching a live capture against the solution key
# ---------------------------------------------------------------------------

def match_and_align(blobs, solution_pieces):
    '''
    greedily match each detected blob to the best-fitting unused solution
    piece by shape (rotation-invariant matchShapes), then compute the exact
    rotation delta needed to bring it into its solved orientation and the
    translation needed to move it to its target position.

    matchShapes (Hu moments) is the primary matcher, same as the wiggly
    puzzle type -- it's already rotation/scale-invariant and works fine on
    jigsaw silhouettes. edge_types (tab/blank/flat per side) is carried
    through into the output for visibility/debugging but doesn't currently
    constrain matching; incorporating it as a secondary signal (e.g.
    requiring a cyclic match of edge_types once rotation_delta is known)
    would add robustness against near-identical piece shapes, but needs
    real scrambled-piece test data to validate before relying on it.
    '''
    remaining = list(solution_pieces)
    matched = []

    for blob in blobs:
        best_piece, best_shape_score = None, float("inf")
        for sp in remaining:
            score = cv2.matchShapes(
                blob["contour"].astype(np.float32),
                sp["reference_contour"].astype(np.float32),
                cv2.CONTOURS_MATCH_I1, 0.0,
            )
            if score < best_shape_score:
                best_piece, best_shape_score = sp, score

        if best_piece is None:
            continue
        remaining.remove(best_piece)

        detected_angle = piece_orientation(blob["contour"])
        rotation_delta = (best_piece["reference_angle"] - detected_angle + np.pi) % (2 * np.pi) - np.pi

        translation = [
            round(best_piece["target_centroid"][0] - blob["centroid"][0], 2),
            round(best_piece["target_centroid"][1] - blob["centroid"][1], 2),
        ]

        matched.append({
            "id": best_piece["id"],
            "current_centroid": blob["centroid"],
            "target_centroid": best_piece["target_centroid"],
            "translation": translation,
            "rotation_delta_rad": round(float(rotation_delta), 4),
            "area": blob["area"],
            "shape_match_score": round(float(best_shape_score), 4),
            "current_edge_types": blob["edge_types"],
            "target_edge_types": best_piece["edge_types"],
            # contours are stored centroid-centered; re-add the (already
            # solved-orientation) centroid to get absolute-position polygons
            # for debug visualization (see main.plot_moves)
            "current_contour": (blob["contour"] + blob["centroid"]).tolist(),
            "target_contour": (best_piece["reference_contour"] + best_piece["target_centroid"]).tolist(),
        })

    if remaining:
        # detected fewer pieces than the solution expects (occlusion, piece off-table, etc)
        matched_ids = {m["id"] for m in matched}
        missing = [sp["id"] for sp in solution_pieces if sp["id"] not in matched_ids]
        print(f"Warning: {len(missing)} solution piece(s) not matched to any detected blob: {missing}")

    return matched


def process_frame(image, solution_path=SOLUTION_KEY_PATH, min_area=200, output_path=CURRENT_PIECES_PATH):
    '''
    full live-capture pipeline: clean the photo, detect piece blobs, match
    each to its solution-key counterpart, and save the moves needed
    (translation + rotation) to current_pieces.json.
    '''
    bw = clean(image)
    blobs = detect_blobs(bw, min_area=min_area)
    solution_pieces = load_solution(solution_path)
    matched = match_and_align(blobs, solution_pieces)

    with open(output_path, "w") as f:
        json.dump({"pieces": matched}, f, indent=4)

    return matched


if __name__ == "__main__":
    # Example usage:
    #
    # 1) One-time: build the solution key from a photo/render of the
    #    solved puzzle.
    # answer_key_img = cv2.imread("j4.png")
    # build_solution_key(answer_key_img, apply_calibration=False)
    #
    # 2) Each time you want to solve: capture the current table state and
    #    compute the moves.
    # live_img = cv2.imread("table_capture.jpg")
    # moves = process_frame(live_img)
    # for m in moves:
    #     print(m)
    pass
