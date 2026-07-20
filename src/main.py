
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
    '''
    Hello programmer! This main function does the following:
    1. captures current image
    1. calibrates the image to robot space transform matrix if RECALIBRATION=True
    2. identifies the solution to PUZZLE_PATH and saves this solution as f"solution_key_{PUZZLE_PATH}.json" 
    3. given the current image of the pieces, identify the pieces and match them to the solution key
    4. execute the robot moves to assemble the puzzle

    Things I am Worried About:
    1. noise in the image
    2. calibration error (so, add a bit of margin for each puzzle piece)
    3. singularities with rotating the robot gripper
    
    Architecture:
    - Use classes for all objects that can have multiple kinds of api or type. For instance, can have jigsaw or wavy piece puzzles, so class Puzzle can have an id config where it executes certain functions for a certain type of puzzle
    - the following groups should be black boxes to each other: {Puzzle, Gripper, Robot}
    
    '''
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