
'''
1. puzzleA.png is a white background with thin black lines indicated puzzle border. fn "solution_blob_finder" returns each shape in maehtmatical description for bloc analysis
2. fn "solution_
'''
import robot_control
from puzzle import Puzzle


def main():
    puzzle = Puzzle(puzzlepath="media/puzzles/Puzzle_12.png")

    puzzle.calibrate() # after this step, calibration config.json exists else prog shld panic
    print(f"loaded config: {puzzle.config}")

    puzzle.build_answerkey()
    print(f"solution key built: {len(puzzle.solution_pieces)} pieces")

    moves = puzzle.solve_current()
    if moves is not None:
        print(f"computed {len(moves)} piece moves -> configs/current_pieces.json")
        robot_control.execute_moves(moves)


if __name__ == "__main__":
    main()