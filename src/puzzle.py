
import os

import cv2

from calibrate_static import calibrate
from camera_api import Camera
import puzzle_solver
from const import *



class Puzzle:
    '''
    make class for GUI convenience
    '''
    def __init__(self, puzzlepath="media/puzzles/puzzle.png", recalibrate=0, simulated=SIMULATION):

        # filepath to selected puzzle
        self.puzzlepath : str = puzzlepath
        self.answerkey_img = cv2.imread(puzzlepath)

        # bool whether shld recalibr or not
        self.recalibrate : bool = recalibrate

        self.config = None
        self.solution_pieces = None
        self.moves = None

        # acquire live image of the current table state
        self.camera = Camera(simulated=simulated)
        self.capture_path = CAPTURE_PATH
        self.camera.capture(self.capture_path)

    def calibrate(self):
        '''
        populate self.config with the path to the calibration file
        '''
        if self.recalibrate: # if recalibrate = 1
            self.config = calibrate() # runs the click-4-corners UI, saves config.json
        else: # if recalibrate=0:
            self.config = CONFIG_PATH

        if not os.path.exists(self.config):
            raise FileNotFoundError(
                f"No calibration found at {self.config} -- run with recalibrate=1 first"
            )

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
        current_img = cv2.imread(self.capture_path)
        if current_img is None:
            print(f"No live capture found at {self.capture_path} (SIMULATION={SIMULATION}); skipping solve")
            return None

        self.moves = puzzle_solver.process_frame(current_img)
        return self.moves