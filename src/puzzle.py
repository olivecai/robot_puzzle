
import importlib
import os

import cv2

from calibrate_static import calibrate
from camera_api import Camera
from const import *

'''
treat class Puzzle like a Trait.

Each puzzle type (wiggly, jigsaw, ...) is registered in const.PUZZLE_CONFIGS
with its own solver module/answer-key path/solved size, and its
puzzles/<type>/puzzle_solver.py implements the shared solving
interface (build_solution_key, process_frame, clean, ...) used by
_PuzzleImpl below.

Then we have a sort of rule: IF you want to integrate your custom puzzle, THEN its puzzle_solver.py must satisfy that public interface, and it must be registered in const.PUZZLE_CONFIGS.
'''

class Puzzle:
    def __init__(self, puzzletype=PUZZLE_TYPE, puzzlepath=PUZZLE_PATH, recalibrate=RECALIBRATE, simulated=SIMULATION, config_path=CONFIG_PATH):

        puzzle_module = importlib.import_module(PUZZLE_CONFIGS[puzzletype]["module"])
        # set self.puzzle, but actually we dont strictly need to access this field
        self.puzzle = _PuzzleImpl(puzzle_module, puzzlepath=puzzlepath, recalibrate=recalibrate, simulated=simulated)

    def build_answerkey(self):
        self.puzzle.build_answerkey()

    def solve_current(self):
        return self.puzzle.solve_current()

    def get_solution_pieces(self):
        return(self.puzzle.solution_pieces)


class _PuzzleImpl:
    '''
    make class for GUI convenience

    thin wrapper around a puzzle_solver module (wiggly/jigsaw/...); works
    for any type registered in const.PUZZLE_CONFIGS since they all share
    the same solver interface.
    '''
    def __init__(self, puzzle_solver, puzzlepath=PUZZLE_PATH, recalibrate=RECALIBRATE, simulated=SIMULATION):

        self.puzzle_solver = puzzle_solver
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
        self.solution_pieces = self.puzzle_solver.build_solution_key(
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
        self.moves = self.puzzle_solver.process_frame(current_img)
        return self.moves
