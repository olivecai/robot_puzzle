import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from configs.robot import *

from const import *

'''
ideally this shld work regardless of the type of robot
'''

def robot_message_send(command: str, ip=ROBOT_IP, port=PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ip, port))
        s.send((command + "\n").encode("utf-8"))


if __name__ == "__main__":
    # Example: move to a joint position
    move_cmd = "movej([0, -1.57, 1.57, -1.57, -1.57, 0], a=1.0, v=0.5)"
    robot_message_send(move_cmd)