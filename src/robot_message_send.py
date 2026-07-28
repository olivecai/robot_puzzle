import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from configs.robot import *

from const import *

def robot_message_send(command: str, ip=ROBOT_IP, port=PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Connecting IP and port...")
        s.connect((ip, port))
        print("Sending...")
        s.send((command + "\n").encode("utf-8"))


if __name__ == "__main__":
   pass