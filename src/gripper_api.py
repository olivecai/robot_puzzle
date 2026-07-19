'''
this file contains black box functions for main.py by wrapping api specific functions to output generic types.
this way, if you change the gripper in the future all you need to edit is this file and strictly nothing else

currently targets a Schmalz Cobotpump suction cup via a single digital output.
'''

import os
import sys

# configs/ lives one level above src/, at the repo root -- put the repo root
# on sys.path so `configs` is importable regardless of cwd or entry point
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from configs.robot import VACUUM_DIGITAL_OUT
from send_UR_msg import send_urscript
from const import *

class Gripper:
    def __init__(self, simulated=SIMULATION):
        self.simulated = simulated
        self.state = None

    def on(self):
        self._set(True)

    def off(self):
        self._set(False)

    def _set(self, state):
        command = f"set_digital_out({VACUUM_DIGITAL_OUT}, {state})"
        if self.simulated:
            print(f"logging: gripper_api SIMULATED RUN -- {command}")
        else:
            send_urscript(command)
        self.state = state