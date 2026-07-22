import json

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

import robot_control
from const import *

# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def principal_angle(cnt):
    '''
    compute a shape's orientation as a single angle in [-pi, pi), using PCA
    for the axis and third-moment skew to resolve the 180-degree ambiguity
    PCA alone can't distinguish. cnt must already be centered on its own
    centroid (mean-subtracted). Requires an asymmetric (non-mirror-symmetric)
    shape to give a stable result.

    NOTE: kept for reference/comparison, but match_and_align no longer uses
    this for rotation_delta_rad -- see rotation_search_cost, which finds the
    best-fit rotation directly from contour overlay and is far more robust
    for pieces whose global shape statistics (Hu moments / PCA skew) are
    ambiguous or near-symmetric.
    '''
    pts = cnt.astype(np.float64)
    cov = np.cov(pts.T)
    eigvals, eigvecs = np.linalg.eigh(cov)  # ascending order
    principal = eigvecs[:, -1]  # eigenvector for the largest eigenvalue

    angle = np.arctan2(principal[1], principal[0])

    proj = pts @ principal
    skew = np.mean(proj ** 3)
    if skew < 0:
        angle += np.pi

    return (angle + np.pi) % (2 * np.pi) - np.pi


def contour_of_mask(mask):
    '''return the largest external contour found in a binary mask, as an (N,2) float array.'''
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt = max(contours, key=cv2.contourArea)
    return cnt.reshape(-1, 2).astype(np.float64)


def warp(image, config_path=CONFIG_PATH):
    with open(config_path) as f:
        config = json.load(f)
    M = np.array(config["M"], dtype=np.float64)
    output_size = tuple(config["output_size"])
    return cv2.warpPerspective(
        image, M, output_size,
        flags=cv2.INTER_CUBIC,   # sharper resampling than the default INTER_LINEAR
        borderValue=(255, 255, 255)
    )

# ---------------------------------------------------------------------------
# Live-capture pipeline: pieces are solid black blobs on a white table
# ---------------------------------------------------------------------------

def clean(image, config_path=CONFIG_PATH, upsample_factor=2, sharpen_amount=1.5):
    '''
    crop and binarize the table image for piece detection. Thresholding is
    done at an upsampled resolution (then downscaled back) so corners come
    out sharper than thresholding at the original camera resolution would
    give -- low-res corners have nowhere to "round off" to at 2x-4x scale.
    Returns a single bw image at the ORIGINAL resolution; callers don't
    need to know upsampling happened internally.
    '''
    cropped = warp(image, config_path)
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    orig_h, orig_w = gray.shape

    gray_up = cv2.resize(gray, None, fx=upsample_factor, fy=upsample_factor,
                          interpolation=cv2.INTER_CUBIC)

    blurred = cv2.GaussianBlur(gray_up, (0, 0), 3)
    sharpened = cv2.addWeighted(gray_up, 1 + sharpen_amount, blurred, -sharpen_amount, 0)
    sharpened = np.clip(sharpened, 0, 255).astype(np.uint8)

    _, bw_up = cv2.threshold(sharpened, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    bw = cv2.resize(bw_up, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)
    _, bw = cv2.threshold(bw, 127, 255, cv2.THRESH_BINARY)  # re-binarize after resize interpolation
    return bw


def detect_blobs(bw, min_area=200):
    '''
    find each piece blob in a cleaned (binary) image. returns each blob's
    centroid (in full-image coordinates) and its contour re-centered on
    that centroid (so shape comparison is position-independent).

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
        })

    return regions


def fit_scale_and_offset(src_size, area_px):
    '''
    uniform, aspect-preserving scale + offset that centers a src_size=(w, h)
    frame inside area_px=(x0, y0, x1, y1). returns (scale, (offset_x,
    offset_y)) such that mapped_point = original_point * scale + offset.

    uniform scale (same factor on x and y) is required so that angles --
    principal_angle/rotation_delta_rad and matchShapes -- aren't distorted;
    an independent x/y scale would stretch piece shapes.
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
    (its shape in solved orientation), and reference angle (its "solved"
    rotation -- kept for reference/debugging, no longer used as the
    zero-point for rotation deltas; see match_and_align/rotation_search_cost).

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
            "reference_angle": round(float(principal_angle(region["contour"])), 4),
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
# Contour-overlay rotation search: replaces Hu-moment (cv2.matchShapes)
# matching. Hu moments compress a whole contour into 7 global statistics,
# which are often too coarse to tell apart jigsaw/wiggly pieces that share
# a similar overall blob shape but differ in tab/blank position -- this
# directly overlays actual contour points at many candidate rotations and
# scores how tightly they align, which is far more discriminative. The
# best-fit rotation found here is also used directly as rotation_delta_rad,
# replacing principal_angle (which assumes an asymmetric shape and can be
# thrown off by rounded corners or near-symmetric pieces).
# ---------------------------------------------------------------------------

def resample_contour(cnt, n_points=150):
    '''resample a closed contour to n_points evenly spaced by arc length.'''
    pts = np.vstack([cnt, cnt[0]])  # close the loop
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_length = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    total_length = cum_length[-1]

    if total_length == 0:
        return np.tile(cnt[0], (n_points, 1))

    sample_distances = np.linspace(0, total_length, n_points, endpoint=False)
    idxs = np.searchsorted(cum_length, sample_distances, side="right") - 1
    idxs = np.clip(idxs, 0, len(seg_lengths) - 1)

    seg_starts = pts[idxs]
    seg_vecs = diffs[idxs]
    seg_lens = seg_lengths[idxs]
    t = np.where(seg_lens > 0, (sample_distances - cum_length[idxs]) / np.where(seg_lens > 0, seg_lens, 1), 0)

    return seg_starts + t[:, None] * seg_vecs


def rotate_points(pts, theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T


def rotation_search_cost(blob_pts, ref_tree, coarse_step_deg=5, fine_step_deg=0.5):
    '''
    find the rotation (radians) that best overlays blob_pts onto the
    reference contour represented by ref_tree (a cKDTree built from the
    reference's resampled points), and the resulting mean nearest-neighbor
    distance at that rotation (the match cost). Coarse-to-fine: scan every
    coarse_step_deg first, then refine around the best coarse angle at
    fine_step_deg for a more precise rotation estimate without scanning
    the full 360 degrees at fine resolution.
    '''
    best_angle, best_dist = 0.0, float("inf")

    for deg in np.arange(0, 360, coarse_step_deg):
        theta = np.radians(deg)
        rotated = rotate_points(blob_pts, theta)
        d, _ = ref_tree.query(rotated)
        dist = d.mean()
        if dist < best_dist:
            best_dist, best_angle = dist, theta

    coarse_deg = np.degrees(best_angle)
    for deg in np.arange(coarse_deg - coarse_step_deg, coarse_deg + coarse_step_deg, fine_step_deg):
        theta = np.radians(deg)
        rotated = rotate_points(blob_pts, theta)
        d, _ = ref_tree.query(rotated)
        dist = d.mean()
        if dist < best_dist:
            best_dist, best_angle = dist, theta

    # wrap into [-pi, pi)
    best_angle = (best_angle + np.pi) % (2 * np.pi) - np.pi
    return float(best_dist), float(best_angle)


def build_piece_trees(solution_pieces, n_points=150):
    '''precompute a resampled-point cKDTree for each solution piece, reused across all blobs.'''
    trees = []
    for sp in solution_pieces:
        pts = resample_contour(sp["reference_contour"], n_points)
        trees.append(cKDTree(pts))
    return trees


def build_cost_matrix(blobs, solution_pieces, n_points=150, coarse_step_deg=5, fine_step_deg=0.5):
    '''
    full (n_blobs x n_pieces) cost matrix using rotation-search contour
    overlay distance, plus the best-fit rotation for each pair (needed
    directly as rotation_delta_rad for whichever assignment is chosen).
    '''
    trees = build_piece_trees(solution_pieces, n_points=n_points)
    n_blobs = len(blobs)
    n_pieces = len(solution_pieces)

    cost = np.zeros((n_blobs, n_pieces))
    angle = np.zeros((n_blobs, n_pieces))

    for i, blob in enumerate(blobs):
        blob_pts = resample_contour(blob["contour"], n_points=n_points)
        for j, tree in enumerate(trees):
            dist, theta = rotation_search_cost(blob_pts, tree, coarse_step_deg, fine_step_deg)
            cost[i, j] = dist
            angle[i, j] = theta

    return cost, angle


def debug_print_cost_matrix(blobs, solution_pieces, top_n=3, n_points=150,
                             coarse_step_deg=5, fine_step_deg=0.5):
    '''
    prints each blob's top_n closest solution-piece candidates by
    rotation-search contour distance, sorted best-first. Run this before
    trusting a matching fix -- if the top 2 candidates for a blob are very
    close, that confirms ambiguity remains even with this stronger metric
    (in which case the underlying blob contour itself may be noisy --
    check for occlusion, touching pieces, or a still-imperfect mask).
    '''
    cost, _ = build_cost_matrix(blobs, solution_pieces, n_points, coarse_step_deg, fine_step_deg)
    for i in range(len(blobs)):
        scores = sorted(
            [(solution_pieces[j]["id"], round(float(cost[i, j]), 4)) for j in range(len(solution_pieces))],
            key=lambda x: x[1],
        )
        print(f"blob {i}: top {top_n} candidates -> {scores[:top_n]}")


def match_and_align(blobs, solution_pieces, n_points=150, coarse_step_deg=5, fine_step_deg=0.5):
    '''
    globally optimal one-to-one matching between detected blobs and
    solution pieces via the Hungarian algorithm on a rotation-search
    contour-overlay cost matrix (see build_cost_matrix/rotation_search_cost).
    rotation_delta_rad comes directly from the same rotation search that
    produced the winning match, rather than from principal_angle -- this
    also fixes rotation accuracy for pieces where global-shape-based angle
    estimation (PCA + skew) was unreliable.
    '''
    n_blobs = len(blobs)
    n_pieces = len(solution_pieces)

    cost, angle = build_cost_matrix(blobs, solution_pieces, n_points, coarse_step_deg, fine_step_deg)
    row_idx, col_idx = linear_sum_assignment(cost)

    matched = []
    view_solution = []
    for i, j in zip(row_idx, col_idx):
        blob = blobs[i]
        best_piece = solution_pieces[j]
        best_shape_score = cost[i, j]
        rotation_delta = angle[i, j]

        translation = [
            round(best_piece["target_centroid"][0] - blob["centroid"][0], 2),
            round(best_piece["target_centroid"][1] - blob["centroid"][1], 2),
        ]

        view_solution.append({
             "id": best_piece["id"],
            "current_centroid": blob["centroid"],
            "target_centroid": best_piece["target_centroid"],
            "translation": translation,
            "rotation_delta_rad": round(float(rotation_delta), 4),
            "area": blob["area"],
            "shape_match_score": round(float(best_shape_score), 4),
        })
        matched.append({
            "id": best_piece["id"],
            "current_centroid": blob["centroid"],
            "target_centroid": best_piece["target_centroid"],
            "translation": translation,
            "rotation_delta_rad": round(float(rotation_delta), 4),
            "area": blob["area"],
            "shape_match_score": round(float(best_shape_score), 4),
            "current_contour": (blob["contour"] + blob["centroid"]).tolist(),
            "target_contour": (best_piece["reference_contour"] + best_piece["target_centroid"]).tolist(),
        })

    if n_blobs != n_pieces:
        matched_ids = {m["id"] for m in matched}
        missing = [sp["id"] for sp in solution_pieces if sp["id"] not in matched_ids]
        print(f"Warning: {n_blobs} detected blob(s) vs {n_pieces} solution piece(s) -- "
              f"unmatched solution piece(s): {missing}")
    

    with open("PREVIEW_SOLUTION.json", "w") as f:
        json.dump({"pieces": view_solution}, f, indent=4)

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

    # Uncomment to inspect match ambiguity before trusting the assignment:
    # debug_print_cost_matrix(blobs, solution_pieces)

    matched = match_and_align(blobs, solution_pieces)

    with open(output_path, "w") as f:
        json.dump({"pieces": matched}, f, indent=4)

    return matched

def check_symmetry_ambiguity(blob, matched_piece, n_points=150, coarse_step_deg=5, fine_step_deg=0.5):
    '''
    for one matched (blob, solution_piece) pair, compares the winning
    rotation's cost against the cost at the same rotation + 180 degrees.
    If these are close, the piece's silhouette is near-symmetric and the
    rotation search may have picked the wrong member of an ambiguous pair.
    '''
    blob_pts = resample_contour(blob["contour"], n_points)
    ref_pts = resample_contour(matched_piece["reference_contour"], n_points)
    tree = cKDTree(ref_pts)

    dist_a, angle_a = rotation_search_cost(blob_pts, tree, coarse_step_deg, fine_step_deg)

    best_flip_dist = float("inf")
    for deg in np.arange(175, 185, fine_step_deg):
        rotated = rotate_points(blob_pts, np.radians(deg))
        d, _ = tree.query(rotated)
        if d.mean() < best_flip_dist:
            best_flip_dist = d.mean()

    ratio = best_flip_dist / dist_a if dist_a > 0 else float("inf")
    print(f"chosen angle: {np.degrees(angle_a):.1f} deg, cost {dist_a:.4f}   "
          f"180-flipped cost: {best_flip_dist:.4f}   ratio: {ratio:.2f}")
    return dist_a, best_flip_dist


def debug_check_flipped_pieces(image, flipped_ids, solution_path=SOLUTION_KEY_PATH, min_area=200):
    '''
    runs the real detection+matching pipeline on `image`, then for each
    piece id in flipped_ids, prints its symmetry-ambiguity check. Use this
    to confirm (or rule out) near-180-degree-symmetric silhouettes as the
    cause of specific pieces landing rotated wrong.
    '''
    bw = clean(image)
    blobs = detect_blobs(bw, min_area=min_area)
    solution_pieces = load_solution(solution_path)
    matched = match_and_align(blobs, solution_pieces)

    solution_by_id = {sp["id"]: sp for sp in solution_pieces}
    blobs_by_centroid = {tuple(b["centroid"]): b for b in blobs}

    for m in matched:
        if m["id"] not in flipped_ids:
            continue
        blob = blobs_by_centroid[tuple(m["current_centroid"])]
        piece = solution_by_id[m["id"]]
        print(f"--- piece id {m['id']} ---")
        check_symmetry_ambiguity(blob, piece)
        
if __name__ == "__main__":
    # Example usage:
    #
    # 1) One-time: build the solution key from a photo/render of the
    #    solved puzzle.
    # answer_key_img = cv2.imread("Puzzle_12.png")
    # build_solution_key(answer_key_img, apply_calibration=False)
    #
    # 2) Each time you want to solve: capture the current table state and
    #    compute the moves.
    # live_img = cv2.imread("table_capture.jpg")
    # moves = process_frame(live_img)
    # for m in moves:
    #     print(m)
    pass