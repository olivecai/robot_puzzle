from puzzle_solver import build_solution_key, clean, detect_blobs, load_solution, match_and_align
from test_puzzle_solve import synthesize_scrambled_capture
import robot_control, cv2, json

img = cv2.imread("media/puzzles/Puzzle_12.png")
pieces = build_solution_key(img, apply_calibration=False)
canvas, _ = synthesize_scrambled_capture(pieces)
with open("configs/config.json", "w") as f:
    json.dump({"M": [[1,0,0],[0,1,0],[0,0,1]], "output_size": list(canvas.shape[::-1])}, f)

bw = clean(cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR))
matched = match_and_align(detect_blobs(bw, min_area=200), load_solution())
robot_control.execute_moves(matched, dry_run=True)
