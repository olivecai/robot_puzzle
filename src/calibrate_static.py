import json
import os

import cv2
import numpy as np

IMAGE_PATH = "media/captures/Capture.jpg"
CONFIG_PATH = "configs/config.json"

points = []
frame_w = None
frame_h = None


def calibrate_perspective(pts):
    global M, output_size
    if len(pts) != 4:
        return False

    # Convert points to numpy array
    rect = np.array(pts, dtype="float32")

    # Order the points: top-left, top-right, bottom-right, bottom-left
    pts = rect.reshape(4, 2)
    sorted_pts = np.zeros((4, 2), dtype="float32")

    # Sum of coordinates
    s = pts.sum(axis=1)
    sorted_pts[0] = pts[np.argmin(s)]  # Top-left
    sorted_pts[2] = pts[np.argmax(s)]  # Bottom-right

    # Difference of coordinates
    diff = np.diff(pts, axis=1)
    sorted_pts[1] = pts[np.argmin(diff)]  # Top-right
    sorted_pts[3] = pts[np.argmax(diff)]  # Bottom-left

    # Define destination points for perspective transform (axis-aligned rectangle)
    (tl, tr, br, bl) = sorted_pts
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    dst = np.array(
        [[0, 0], [maxWidth - 1, 0], [maxWidth - 1, maxHeight - 1], [0, maxHeight - 1]],
        dtype="float32",
    )

    # Compute perspective transform matrix
    M = cv2.getPerspectiveTransform(sorted_pts, dst)
    output_size = (maxWidth, maxHeight)
    return True


def mouse_callback(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        rect = cv2.getWindowImageRect("Calibration Image")
        if rect is not None:
            img_x = x - rect[0]
            img_y = y - rect[1]
            if 0 <= img_x < rect[2] and 0 <= img_y < rect[3]:
                scale_x = frame_w / rect[2]
                scale_y = frame_h / rect[3]
                original_x = int(img_x * scale_x)
                original_y = int(img_y * scale_y)
                points.append((original_x, original_y))
                print(f"Point {len(points)}: ({original_x}, {original_y})")


def apply_perspective_transform(frame):
    global M, output_size
    if M is None or output_size is None:
        return None
    # Apply perspective warp
    warped = cv2.warpPerspective(frame, M, output_size)
    return warped


def calibrate():
    global points, frame_w, frame_h

    frame = cv2.imread(IMAGE_PATH)
    if frame is None:
        print(f"Could not load image: {IMAGE_PATH}")
        return
    frame_h, frame_w = frame.shape[:2]

    print("Click 4 corners of the white rectangle (in order), then press 's' to calibrate.")
    print("Press 'r' to reset points, 'q' to quit.")

    cv2.namedWindow("Calibration Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Calibration Image", 1920, 1080)
    cv2.setMouseCallback("Calibration Image", mouse_callback)

    while True:
        display_frame = frame.copy()
        rect = cv2.getWindowImageRect("Calibration Image")
        scale_x = rect[2] / frame_w
        scale_y = rect[3] / frame_h

        for i, pt in enumerate(points):
            display_pt = (int(pt[0] * scale_x), int(pt[1] * scale_y))
            cv2.circle(display_frame, display_pt, 10, (0, 255, 0), 2)
            cv2.circle(display_frame, display_pt, 5, (0, 255, 0), -1)
            cv2.putText(
                display_frame,
                str(i + 1),
                (display_pt[0] + 15, display_pt[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        if len(points) == 4:
            display_points = [
                (int(pt[0] * scale_x), int(pt[1] * scale_y)) for pt in points
            ]
            cv2.polylines(
                display_frame, [np.array(display_points)], True, (255, 0, 0), 2
            )

        cv2.imshow("Calibration Image", display_frame)

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
            if calibrate_perspective(points):
                os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
                config_data = {
                    "points": points,
                    "M": M.tolist(),
                    "output_size": output_size,
                }
                with open(CONFIG_PATH, "w") as f:
                    json.dump(config_data, f, indent=4)
                print(f"Calibration data saved to {CONFIG_PATH}")

                transformed = apply_perspective_transform(frame)
                cv2.namedWindow("Transformed Puzzle", cv2.WINDOW_NORMAL)
                cv2.imshow("Transformed Puzzle", transformed)
            else:
                print("Calibration failed")

    cv2.destroyAllWindows()
    return CONFIG_PATH


if __name__ == "__main__":
    calibrate()
