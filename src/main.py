
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
import robot_control
from puzzle import Puzzle
from const import *

def main():
    puzzle = Puzzle(puzzlepath=PUZZLE_PATH)

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