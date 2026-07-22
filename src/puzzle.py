
import os

import cv2

from calibrate_static import calibrate
from camera_api import Camera
import media.puzzles.wiggly.puzzle_solver as puzzle_solver
from const import *

'''
treat class Puzzle like a Trait.

If you have a type of Puzzle like WigglyPuzzle or JigsawPuzzle etc, you can implement its solving logic etc and then expose the traits dictated by Puzzle

Then we have a sort of rule: IF you want to integrate your custom puzzle, THEN it must satisfy the public traits of Puzzle.
'''

class Puzzle:
    def __init__(self, puzzletype = PUZZLE_TYPE, puzzlepath=PUZZLE_PATH, recalibrate=RECALIBRATE, simulated=SIMULATION, config_path=CONFIG_PATH):
        
        # set self.puzzle, but actually we dont strictly need to access this field
        if PUZZLE_TYPE == WIGGLY:
            self.puzzle=WigglyPuzzle(puzzlepath=puzzlepath, recalibrate=recalibrate, simulated=simulated)
        
    def build_answerkey(self):
        self.puzzle.build_answerkey()
    
    def solve_current(self):
        return self.puzzle.solve_current()

    def get_solution_pieces(self):
        return(self.puzzle.solution_pieces)    


class WigglyPuzzle:
    '''
    make class for GUI convenience
    '''
    def __init__(self, puzzlepath=PUZZLE_PATH, recalibrate=RECALIBRATE, simulated=SIMULATION):

        # filepath to selected puzzle
        self.puzzlepath : str = puzzlepath
        self.answerkey_img = cv2.imread(puzzlepath)
        self.simulated=simulated

        self.solution_pieces = None
        self.moves = None



    def build_answerkey(self):
        '''
        segment the answer-key template into configs/solution_key.json
        '''
        self.solution_pieces = puzzle_solver.build_solution_key(
            self.answerkey_img, apply_calibration=False
        )

    def solve_current(self):
        '''
        capture -> clean -> detect -> match against the solution key;
        returns the translation/rotation each detected piece needs to reach
        its solved position
        '''
        current_img = cv2.imread(CAPTURE_PATH)
        if current_img is None:
            print(f"No live capture found at {CAPTURE_PATH} (SIMULATION={self.simulated}); skipping solve")
            return None

        print("LOGGING puzzle::solve_current; processing frame now...")
        self.moves = puzzle_solver.process_frame(current_img)
        return self.moves
    
# class JigsawPuzzle:
#     def calibrate
