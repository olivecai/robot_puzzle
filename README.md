# robot_puzzle
Repository for the UR arm to solve puzzles using CV

## brainstorming

PC Host is plugged into 


Core files
puzzle_calibration.py — one-time camera setup

Opens the webcam at 4K, lets you click the 4 corners of the puzzle work-area (the "pickup frame") on a captured frame.
Computes a perspective-transform matrix (cv2.getPerspectiveTransform) that un-warps the camera's angled view into a top-down rectangle.
Saves points, the matrix M, and output_size to configs/config.json — this is the calibration every later run relies on.
puzzle_solution.py — offline pass over a solved reference puzzle image

Run manually as python puzzle_solution.py path/to/solved.png 24.
Thresholds/inverts the image, finds contours (cv2.findContours with RETR_LIST), and keeps the N largest as the individual puzzle pieces.
For each piece computes: centroid, area, perimeter, Hu moments (rotation/scale/translation-invariant shape descriptors), and a PCA-based orientation (get_robust_orientation) that finds the piece's principal axis and disambiguates 180° flips using third-moment skewness.
Writes all of this to configs/Puzzle_{N}.json — the "ground truth" solution — plus per-piece colored mask PNGs (configs/piece_{id}_mask.png) used later for IoU matching.
puzzle_gui.py — the main CustomTkinter application, does the real-time work

Loads calibration + the target solution JSON.
Captures a frame (or continuously in "Live Mode"), applies the saved perspective warp, stretches to 16:10, thresholds it, and finds contours for whatever loose pieces are currently visible under the camera — same Hu-moment + PCA-orientation pipeline as above.
Matching: builds a cost matrix of Hu-moment distances between every detected piece and every target piece, then solves optimal assignment with scipy.optimize.linear_sum_assignment (Hungarian algorithm).
Orientation disambiguation: since Hu moments and PCA angle can't tell a piece from its 180°-rotated twin, it renders the detected piece's mask, compares IoU against the target mask both normally and rotated 180°, and picks whichever orientation overlaps better.
Pose output: converts detected/target centroids from pixels to millimeters using known physical frame sizes (pickup tray 520×325mm, target A4 297×210mm), and for each piece reports a translation (mm) and rotation (degrees) — i.e. "move this piece by (Δx, Δy) mm and rotate it θ° to complete the puzzle."
GUI shows four synced views (raw camera, warped, threshold mask, detected pieces) plus the solution image with an overlay; hovering over a detected piece highlights where it belongs and prints the exact translate/rotate instructions and match confidence (IoU).
Data flow

Solved puzzle photo → puzzle_solution.py → configs/Puzzle_N.json (target shapes/poses)
Webcam corners       → puzzle_calibration.py → configs/config.json (perspective matrix)
Live/scrambled pieces → puzzle_gui.py → warp → threshold → contours → Hu moments/PCA angle
                                     → Hungarian match against Puzzle_N.json → IoU flip-check
                                     → per-piece (Δx, Δy, Δθ) instructions
Notes
No actual robot-arm control code exists in this repo — despite the project name, it currently only guides (GUI overlay + printed translate/rotate values); presumably a robot controller elsewhere consumes the solution_map output.
The README.md is out of date: it references puzzle_capture.py, detect_pieces.py, puzzle_solver.py, live_puzzle_highlighter.py, live_puzzle_solver.py, none of which exist — the actual files are puzzle_calibration.py, puzzle_solution.py, puzzle_gui.py. Worth fixing if you want the docs to be trustworthy.
configs/ currently holds pre-generated data for a specific 24-piece puzzle (piece images, masks, Puzzle_24.json, detected_pieces.png) alongside the live calibration config.json.


## data flow

## todo


1. camera object --> use SDK or OpenCV https://docs.luxonis.com/software-v3/depthai/ 
```
pip install depthai --force-reinstall

cd examples/python
# Run YoloV6 detection example
python3 DetectionNetwork/detection_network.py
# Display all camera streams
python3 Camera/camera_all.py
```
https://docs.luxonis.com/software-v3/depthai/api/python/



## impl

camera acquisition works, run setup_udev_rules.sh for devices perms and then run camera_api.py

