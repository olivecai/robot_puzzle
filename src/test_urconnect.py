import socket
from time import sleep

from robot_message_send import robot_message_send
from configs.robot import ROBOT_IP, PORT

from gripper_api import Gripper

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# print("Trying")
# s.connect((ROBOT_IP, PORT))
# print("Connected")
# # Simple URScript test: pop up a popup on the teach pendant
# script = "popup(\"Hello from laptop\", \"Test\", False, False, blocking=False)\n"
# robot_message_send(script)

# script = """
# def freedrive_test():
#     freedrive_mode([1,1,0,0,0,0])
#     sleep(5.0)
#     end_freedrive_mode()
# end
# freedrive_test()
# """
# robot_message_send(script)

# robot_message_send("""
# def show_pose():
#     p = get_actual_tcp_pose()
#     q = get_actual_joint_positions()
#     popup(to_str(p), "TCP Pose", False, False, blocking=False)
#     popup(to_str(q), "Joint Positions", False, False, blocking=False)
# end
# show_pose()
# """)

from rtde_receive import RTDEReceiveInterface
import json

from configs.robot import ROBOT_IP

rtde_r = RTDEReceiveInterface(ROBOT_IP)

pose = rtde_r.getActualTCPPose()   # [x, y, z, rx, ry, rz]
q = rtde_r.getActualQ()             # joint angles in radians

print("TCP Pose (x, y, z, rx, ry, rz):", pose)
print("Joint Positions (rad):", q)

with open("configs/camera_capture_joint_pose.json", "w") as f:
    json.dump({"tcp_pose": pose, "joints": q}, f, indent=4)

print("Saved to configs/camera_capture_joint_pose.json")

# g = Gripper(simulated=0)
# g.on()
# sleep(2)
# g.off()

print("DONE")