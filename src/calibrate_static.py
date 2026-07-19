'''
Two-stage table calibration.

Stage 1 (perspective): click the 4 corners of the table on a static photo
to compute M, the 3x3 homography that rectifies the raw camera image into
a flat, axis-aligned rectangle (output_size). This only fixes camera
lens/angle distortion -- it says nothing about where the robot is.

Stage 2 (robot frame): on that rectified image, click the fixed physical
markers whose true robot-base XY is already known (const.CALIBRATION_ROBOT_POINTS),
in order, and fit an affine map from rectified-image pixels -> robot-frame
meters. This is what robot_control.py actually needs to send correct
pick/place coordinates.

Both stages' results are saved together into config.json:
    points, M, output_size, pixel_to_robot_affine

NOTE on display scaling: we deliberately never call cv2.getWindowImageRect()
to figure out click-to-original-pixel scaling. That API queries the *live*
window manager state, which can lag behind resizeWindow()/namedWindow() on
some backends (esp. Qt) -- the first click can land before the window has
actually finished laying out, so it silently reads back (0, 0) and every
click gets dropped (this is what caused "Select 4 points first." to loop
forever, alongside the QFont::setPointSizeF warning, which is the same
stale/zero-geometry race surfacing inside Qt's own font code). Instead we
resize the image ourselves to fit SCREEN_WIDTH/SCREEN_HEIGHT (const.py, set
to your actual screen resolution) with cv2.resize() before display, and
compute the scale factor from numbers we already know -- nothing to query,
nothing to race.
'''

import json
import os

import cv2
import numpy as np

from const import CALIBRATION_ROBOT_POINTS, CAPTURE_PATH, CONFIG_PATH, SCREEN_HEIGHT, SCREEN_WIDTH

IMAGE_PATH = CAPTURE_PATH

# leave room for window titlebar/taskbar/dock rather than filling the whole
# screen -- tune if windows still get clipped or run too small on your setup
SCREEN_MARGIN = 0.85
DISPLAY_MAX_W = int(SCREEN_WIDTH * SCREEN_MARGIN)
DISPLAY_MAX_H = int(SCREEN_HEIGHT * SCREEN_MARGIN)

points = []
frame_w = None
frame_h = None
M = None
output_size = None


def make_display(image, max_w=DISPLAY_MAX_W, max_h=DISPLAY_MAX_H):
    '''
    downscale image to fit within max_w x max_h (derived from SCREEN_WIDTH/
    SCREEN_HEIGHT in const.py), preserving aspect ratio. returns
    (display_image, scale) where scale = display_size / original_size
    (so original = display / scale).
    '''
    h, w = image.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)  # never upscale
    if scale == 1.0:
        return image.copy(), 1.0
    display = cv2.resize(image, (int(round(w * scale)), int(round(h * scale))),
                          interpolation=cv2.INTER_AREA)
    return display, scale


# ---------------------------------------------------------------------------
# Stage 1: perspective rectification
# ---------------------------------------------------------------------------

def calibrate_perspective(pts):
    '''fit M/output_size from 4 corner clicks (any order), in original-image coords.'''
    global M, output_size
    if len(pts) != 4:
        return False

    rect = np.array(pts, dtype="float32").reshape(4, 2)
    sorted_pts = np.zeros((4, 2), dtype="float32")

    s = rect.sum(axis=1)
    sorted_pts[0] = rect[np.argmin(s)]  # top-left
    sorted_pts[2] = rect[np.argmax(s)]  # bottom-right

    diff = np.diff(rect, axis=1)
    sorted_pts[1] = rect[np.argmin(diff)]  # top-right
    sorted_pts[3] = rect[np.argmax(diff)]  # bottom-left

    (tl, tr, br, bl) = sorted_pts
    widthA = np.linalg.norm(br - bl)
    widthB = np.linalg.norm(tr - tl)
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.linalg.norm(tr - br)
    heightB = np.linalg.norm(tl - bl)
    maxHeight = max(int(heightA), int(heightB))

    dst = np.array(
        [[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]],
        dtype="float32",
    )

    M = cv2.getPerspectiveTransform(sorted_pts, dst)
    output_size = (maxWidth, maxHeight)
    return True


def apply_perspective_transform(frame):
    if M is None or output_size is None:
        return None
    return cv2.warpPerspective(frame, M, output_size)


def mouse_callback(event, x, y, flags, param):
    '''
    param must be the scale used to produce the currently-displayed image
    (see make_display). x, y are window-local display coords, straight from
    OpenCV -- no window-geometry query needed since the display image size
    is exactly what we chose it to be.
    '''
    global points
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        scale = param
        original_x = int(round(x / scale))
        original_y = int(round(y / scale))
        original_x = max(0, min(original_x, frame_w - 1))
        original_y = max(0, min(original_y, frame_h - 1))
        points.append((original_x, original_y))
        print(f"Point {len(points)}: ({original_x}, {original_y})")


# ---------------------------------------------------------------------------
# Stage 2: pixel (rectified) -> robot-frame XY
# ---------------------------------------------------------------------------

def collect_robot_correspondences(warped_frame):
    '''
    Click the physical markers visible in the RECTIFIED image (shown at a
    fixed, downscaled display size -- see module docstring for why), in the
    same order as const.CALIBRATION_ROBOT_POINTS. Each marker's robot-frame
    XY is read from that list rather than typed in live, so make sure the
    marker you click matches the position you're currently up to -- the
    on-screen prompt below tracks which one is next.
    '''
    robot_pts_known = list(CALIBRATION_ROBOT_POINTS)
    num_points = len(robot_pts_known)
    if num_points < 3:
        raise ValueError(
            f"const.CALIBRATION_ROBOT_POINTS has only {num_points} points -- need at least 3."
        )

    display, scale = make_display(warped_frame)
    clicked = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < num_points:
            orig_x = int(round(x / scale))
            orig_y = int(round(y / scale))
            clicked.append((orig_x, orig_y))
            rx, ry = robot_pts_known[len(clicked) - 1]
            print(f"Marker {len(clicked)}/{num_points} clicked at pixel ({orig_x}, {orig_y}) "
                  f"-> robot ({rx:.3f}, {ry:.3f})")

    win = "Rectified - click robot calibration markers"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, on_click)

    print(f"Click the {num_points} calibration markers on the rectified image, "
          "in the order listed in const.CALIBRATION_ROBOT_POINTS.")
    print("Press 'q' to stop early (need at least 3 points).")

    while len(clicked) < num_points:
        disp = display.copy()
        next_idx = len(clicked)
        if next_idx < num_points:
            rx, ry = robot_pts_known[next_idx]
            cv2.putText(disp, f"Click marker {next_idx + 1}/{num_points}  (robot {rx:.3f}, {ry:.3f})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        for i, p in enumerate(clicked):
            dp = (int(round(p[0] * scale)), int(round(p[1] * scale)))
            cv2.circle(disp, dp, 6, (0, 255, 0), -1)
            cv2.putText(disp, str(i + 1), (dp[0] + 8, dp[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow(win, disp)
        if cv2.waitKey(20) & 0xFF == ord("q"):
            break

    cv2.destroyWindow(win)

    if len(clicked) < 3:
        raise ValueError(f"Only {len(clicked)} marker(s) clicked -- need at least 3.")

    image_pts = np.array(clicked, dtype=np.float64)
    robot_pts = np.array(robot_pts_known[:len(clicked)], dtype=np.float64)

    return image_pts, robot_pts


def fit_pixel_to_robot_transform(image_pts, robot_pts):
    '''returns 2x3 affine A such that [x_robot, y_robot] = A @ [x_px, y_px, 1].'''
    if len(image_pts) < 3:
        raise ValueError("Need at least 3 correspondence points")

    A, _ = cv2.estimateAffine2D(image_pts, robot_pts, method=cv2.RANSAC)
    if A is None:
        raise RuntimeError("Failed to fit pixel -> robot transform")

    homog = np.hstack([image_pts, np.ones((len(image_pts), 1))])
    predicted = (A @ homog.T).T
    residuals = np.linalg.norm(predicted - robot_pts, axis=1)
    print("Per-point residuals (m):", np.round(residuals, 5))
    print("Max residual (m):", round(float(residuals.max()), 5))
    if residuals.max() > 0.005:
        print("WARNING: max residual > 5mm -- check for a bad click or a bad robot-XY reading.")

    return A


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def calibrate():
    global points, frame_w, frame_h

    frame = cv2.imread(IMAGE_PATH)
    if frame is None:
        print(f"Could not load image: {IMAGE_PATH}")
        return None
    frame_h, frame_w = frame.shape[:2]

    display, scale = make_display(frame)

    print("Click 4 corners of the white rectangle (any order), then press 's' to calibrate.")
    print("Press 'r' to reset points, 'q' to quit.")

    win = "Calibration Image"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)  # sized exactly to `display`, no resize race
    cv2.setMouseCallback(win, mouse_callback, param=scale)

    saved_path = None

    while True:
        display_frame = display.copy()

        for i, pt in enumerate(points):
            dp = (int(round(pt[0] * scale)), int(round(pt[1] * scale)))
            cv2.circle(display_frame, dp, 10, (0, 255, 0), 2)
            cv2.circle(display_frame, dp, 5, (0, 255, 0), -1)
            cv2.putText(display_frame, str(i + 1), (dp[0] + 15, dp[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if len(points) == 4:
            display_points = [(int(round(pt[0] * scale)), int(round(pt[1] * scale))) for pt in points]
            cv2.polylines(display_frame, [np.array(display_points)], True, (255, 0, 0), 2)

        cv2.imshow(win, display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("r"):
            points = []
            print("Points reset.")
        elif key == ord("s"):
            if len(points) != 4:
                print("Select 4 points first.")
                continue
            if not calibrate_perspective(points):
                print("Perspective calibration failed")
                continue

            transformed = apply_perspective_transform(frame)
            transformed_display, _ = make_display(transformed)
            cv2.namedWindow("Transformed Puzzle", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("Transformed Puzzle", transformed_display)
            cv2.waitKey(1)

            print("\nNow calibrating pixel -> robot mapping.")
            print("Click each physical marker in order (see const.CALIBRATION_ROBOT_POINTS).")
            image_pts, robot_pts = collect_robot_correspondences(transformed)
            affine = fit_pixel_to_robot_transform(image_pts, robot_pts)

            os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
            config_data = {
                "points": points,
                "M": M.tolist(),
                "output_size": output_size,
                "pixel_to_robot_affine": affine.tolist(),
            }
            with open(CONFIG_PATH, "w") as f:
                json.dump(config_data, f, indent=4)
            saved_path = CONFIG_PATH
            print(f"Calibration data saved to {CONFIG_PATH}")

    cv2.destroyAllWindows()
    return saved_path


if __name__ == "__main__":
    calibrate()