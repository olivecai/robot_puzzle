'''
this file contains black box functions for main.py by wrapping api specific functions to output generic types.
this way, if you change the gripper in the future all you need to edit is this file and strictly nothing else

currently targets a Schmalz Cobotpump suction cup via a single digital output.
'''

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from robot_message_send import robot_message_send
from const import *

VACUUM_DIGITAL_OUT=1
'''
TODO for olivia read this document: https://media.schmalz.com/MAM_Library/Dokumente/Bedienungsanleitung_kurz/30/3030/303001/30300102291/d017758bd2c7_BAK_30.30.01.02291_en-EN.pdf 
'''

class Gripper:
    def __init__(self, grippertype=GRIPPER_TYPE, simulated=SIMULATION):
        if grippertype == DIGITALOUT_URSCRIPT:
            self.gripper = DigitalOutURScriptGripper(simulated=simulated)
    
    def on(self):
        self.gripper.on()

    def off(self):
        self.gripper.off()

class DigitalOutURScriptGripper:
    def __init__(self, simulated=SIMULATION):
        self.simulated = simulated
        self.state = None

    def on(self):
        self._set(True)

    def off(self):
        self._set(False)

    def _set(self, state):
        command = f"set_tool_digital_out({VACUUM_DIGITAL_OUT}, {state})"
        if self.simulated:
            print(f"logging: gripper_api SIMULATED RUN -- {command}")
        else:
            robot_message_send(command)
        self.state = state

class SchmalzGripper:
    def __init__(self, simulated=SIMULATION):
        self.simulated = simulated
        self.state = None

    def on(self):
        pass

    def off(self):
        pass
