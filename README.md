# robot puzzle

## software and connectivity setup:
1. Clone this repository robot_puzzle and cd into robot_puzzle/
1. In a venv, run `pip install -r requirements.txt`
3. Turn on the UR and the UR control tablet.
4. Ensure the UR is in Local mode (tap the icon in the right hand corner of the tablet). If the UR is in Remote Control mode, you will not be able to make modifications in Settings.
5. On the UR control tablet in Settings > Security > Services, enter password to Unlock and enable:
    - Primary Client Interface (port 30001) 
    - RTDEReceiveInterface (port 30004)
6. On the UR control tablet in Settings > System:
    - enable Static Address or DHCP. On your PC, you can ping the UR.
7. On the UR control tablet in the home screen:
    - enable Remote Control mode by tapping the icon in the right hand corner of the tablet from Local mode.
    - You can test the network connection by sending a URScript pop up to the tablet. First open a terminal, activate your venv, and then open a python interpreter. Then run the following python script from robot_puzzle repository root:
        ```
        import socket
        from time import sleep

        from robot_message_send import robot_message_send
        from configs.robot import ROBOT_IP, PORT

        from gripper_api import Gripper

        # print(f"Creating socket. ip=={ROBOT_IP}, port=={PORT}")
        # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # print("Socket created.")
        # print("Connecting...")
        # s.connect((ROBOT_IP, PORT))
        # print("Connected.")
        # print("Sending popup:")
        # script = "popup(\"Hello from laptop\", \"Test\", False, False, blocking=False)\n"
        # print("Sent pop up.")
        # robot_message_send(script)
        ```

8. Plug in the Luxonis camera (USBC) to your PC (USBC or USB).
9. The Luxonis camera does not enumerate as a device to stream through your PC's camera software (ie Cheese) as far as I know... but you can easily test the camera device by running `python src/camera_api.py` in the repository root, which will capture a photo and save it to your PC as `c.capture(f"testphoto_{datetime.now()}.png")`. (*Note*: the venv created from requirements.txt already has depthai==3.8.0 installed.)

## calibration:

In src/const.py:

calibration stage 1
1. first, the robot moves to an optimal pose (configs/camera_capture_joint_pose.json) and then captures an image to path CAPTURE_PATH. 
2. then the user clicks four points of the captured image to map to a planar rectangle; a transformation matrix is generated from image to real world

calibration stage 2
1. the user aks 

Settings > System
- enable Static Address OR DHCP

In the top right hand corner of the tablet
- click the "Local" icon to "Remote Control"
- 

1. camera object --> use SDK or OpenCV https://docs.luxonis.com/software-v3/depthai/ 
```
pip install depthai --force-reinstall

cd examples/python
# Run YoloV6 detection example
python3 DetectionNetwork/detection_network.py
# Display all camera streams
python3 Camera/camera_all.py
```
https://docs.luxonis.com/software-v3/depthai/api/python/



## impl

camera acquisition works, run setup_udev_rules.sh for devices perms and then run camera_api.py

### UR ROBOT 

Settings > Security > Services 
- enter password to Unlock
- enable Primary Client Interface

Settings > System
- enable Static Address OR DHCP

In the top right hand corner of the tablet
- click the "Local" icon to "Remote Control"



### CALIBRATING

- bottom right corner: