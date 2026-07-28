'''
Standalone: takes a puzzle template image (black line-drawing on white,
enclosing puzzle-piece-shaped regions) and outputs a NEW image where every
piece has uniform extra spacing around it -- pieces pushed apart along a
grid (row/column) layout, canvas grown to fit.

Each piece's shape and area are UNCHANGED: every piece's boundary contour
is only ever rigidly translated (shifted as a whole), never scaled or
buffered/inset. Translation cannot alter a shape's geometry or area --
this is guaranteed by construction, not just visually similar.

Usage:
    python add_puzzle_gap.py input.png output.png --gap 20
'''

import argparse

import cv2
import numpy as np


def extract_regions(image, min_area=2000, line_threshold=200, close_iters=1):
    '''find each enclosed piece region in a line-drawing puzzle image.'''
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
            continue  # discard slivers / outer background region

        cx, cy = centroids[label]
        region_binary = np.where(labels == label, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(region_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnt = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)

        regions.append({"centroid": np.array([cx, cy], dtype=np.float64), "area": area, "contour": cnt})

    return regions


def infer_row_col(regions):
    '''
    estimate each region's (row, col) grid index from its centroid,
    assuming a roughly axis-aligned rectangular grid layout (true for
    standard jigsaw templates -- cut from an NxM grid of cells, just with
    wiggly rather than straight cell boundaries).
    '''
    avg_size = np.mean([np.sqrt(r["area"]) for r in regions])
    row_tol = avg_size * 0.6

    order = sorted(regions, key=lambda r: r["centroid"][1])
    rows = [[order[0]]]
    row_y = order[0]["centroid"][1]

    for r in order[1:]:
        y = r["centroid"][1]
        if y - row_y > row_tol:
            rows.append([r])
            row_y = y
        else:
            rows[-1].append(r)
            row_y = np.mean([q["centroid"][1] for q in rows[-1]])

    for row_idx, row in enumerate(rows):
        for col_idx, r in enumerate(sorted(row, key=lambda r: r["centroid"][0])):
            r["row"] = row_idx
            r["col"] = col_idx

    return regions


def add_gap(image_path, output_path, gap_px=15, min_area=2000, line_threshold=200):
    '''
    read image_path, add gap_px of extra spacing per grid row/column
    (pieces pushed apart, shapes/areas unchanged), write result to
    output_path: black background, each piece filled solid white.
    '''
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")
    h, w = image.shape[:2]

    regions = extract_regions(image, min_area=min_area, line_threshold=line_threshold)
    if not regions:
        raise ValueError("No enclosed pieces found -- check line_threshold/min_area against this image.")

    infer_row_col(regions)

    max_row = max(r["row"] for r in regions)
    max_col = max(r["col"] for r in regions)
    new_w = w + max_col * gap_px
    new_h = h + max_row * gap_px

    canvas = np.zeros((new_h, new_w, 3), dtype=np.uint8)  # black background

    for r in regions:
        dx = r["col"] * gap_px
        dy = r["row"] * gap_px
        # rigid translation only -- contour points shifted as-is, no
        # scaling/buffering, so shape and area are exactly preserved
        shifted = (r["contour"] + [dx, dy]).astype(np.int32)
        cv2.fillPoly(canvas, [shifted], color=(255, 255, 255))  # piece filled white

    cv2.imwrite(output_path, canvas)
    print(f"{len(regions)} pieces found, grid {max_row + 1} rows x {max_col + 1} cols, "
          f"canvas {w}x{h} -> {new_w}x{new_h}")
    print(f"Saved: {output_path}")
    return canvas


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Add uniform gap between puzzle pieces (shape/area unchanged).")
    parser.add_argument("input", help="path to input puzzle template image")
    parser.add_argument("output", help="path to write output image")
    parser.add_argument("--gap", type=int, default=15, help="extra spacing per grid row/column, in pixels")
    parser.add_argument("--min-area", type=int, default=2000, help="minimum region area to count as a piece")
    parser.add_argument("--line-threshold", type=int, default=200, help="gray value below which a pixel is a line")
    args = parser.parse_args()

    add_gap(args.input, args.output, gap_px=args.gap, min_area=args.min_area,
            line_threshold=args.line_threshold)