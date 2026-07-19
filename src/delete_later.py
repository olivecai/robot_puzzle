'''
run Puzzle_24_capture.png through gaussian blur
'''
import numpy as np

def add_noise(canvas, stddev):
    '''add per-pixel Gaussian noise to simulate camera sensor noise / uneven lighting.'''
    noise = np.random.normal(0, stddev, canvas.shape)
    return np.clip(canvas.astype(np.float32) + noise, 0, 255).astype(np.uint8)

def test_scramble_and_recover(solution_pieces, seed=RANDOMSEED, noise_stddev=0):
    print("\n=== test_scramble_and_recover ===")
    canvas, ground_truth = synthesize_scrambled_capture(solution_pieces, seed=seed)
    if noise_stddev:
        canvas = add_noise(canvas, noise_stddev)
    cv2.imwrite("scrambled_capture.png", canvas)
    print(f"Wrote scrambled_capture.png ({canvas.shape[1]}x{canvas.shape[0]}, synthetic scrambled pieces)")
