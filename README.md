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

In src/const.py, you have to change a few values depending on your set up, puzzle, and calibration points. By the end of this section, you should have edited the following const values:
- RECALIBRATE
- SIMULATION
- CAPTURE_FRESH 
- PUZZLE_TYPE
- ROBOT_MESSAGE_TYPE
- GRIPPER_TYPE
- CAMERA_TYPE
- PUZZLE_PATH
- CAPTURE_PATH
- CONFIG_PATH
- CURRENT_PIECES_PATH
- SOLUTION_KEY_PATH
- SCREEN_WIDTH
- SCREEN_HEIGHT
- CALIBRATION_ROBOT_POINTS
- SCATTER_AREA_PX
- PUZZLE_SOLVED_SIZE_M_ACTUAL
- PUZZLE_SOLVED_SIZE_M
- ASSEMBLY_OFFSET_M

Instructions:

1. Set RECALIBRATE=1, SIMULATION=0, CAPTURE_FRESH=1.
2. Choose the puzzle you would like to use. Assume you have a black and white photo of your solved puzzle, where black==spaces between puzzle pieces and white==the pieces themselves. Let the location of this puzzle be PUZZLE_PATH.. 
3. Set the *_TYPE constants: PUZZLE_TYPE, ROBOT_MESSAGE_TYPE, GRIPPER_TYPE, CAMERA_TYPE to the type of API you need. For example, we use the DepthAI API to interface with the Luxonis camera, so currently CAMERA_TYPE=DEPTHAI. These constants should not need to be edited unless a device is switched out or a different puzzle/API should be implemented etc.
4. Set the *_PATH constants: PUZZLE_PATH, CAPTURE_PATH are both .png paths (PUZZLE_PATH==location of solved solution puzzle png, CAPTURE_PATH==location of raw camera capture save png). CONFIG_PATH, CURRENT_PIECES_PATH, SOLUTION_KEY_PATH are all .json paths (CONFIG_PATH==location of calibration json, CURRENT_PIECES_PATH==location of current live capture json, SOLUTION_KEY_PATH==location of generated solution to solve current scrambled). 
5. Set the SCREEN_* contants so that the calibration pop up windows to fit inside the dimensions of your PC screen
6. Set CALIBRATION_ROBOT_POINTS: 

    i. On the UR control tablet, set the UR to Local mode. 
    
    ii. Place a calibration board in the robot workspace. Select 4-9 calibration points.

    iii. For each chosen calibration point c{i} do: in freehand mode, move the robot to the location on the calibration board. Then, edit the python file `src/save_tcp_pose.py` line 65 to `POSE_PATH = "configs/calibration/c{i}.json"`.  Run `python src/save_tcp_pose.py`. 

    iv. Run `python configs/calibration/scrape_calib_pnts.py`, which prints output to the terminal. Copy CALIBRATION_ROBOT_POINTS=[...] into const.py

7. Take a picture of the scrambled puzzle workspace (currently camera sees calibration board):

    i. Run the following in a python interpreter to check if the current capture pose saved at configs/camera_capture_joint_pose.json is appropriate for the robot: 
    '''
    import robot_control
    robot_control.go_to_pose_camera_capture()
    '''  
    
    ii. If you would like to overwrite the capture pose: edit the python file `src/save_tcp_pose.py` line 65 to `POSE_PATH = "configs/camera_capture_joint_pose.json"`. Move the arm to a position appropriate for imaging the scrambled puzzle workspace. Write the pose by running `python src/save_tcp_pose.py`

    iii. Ensure the robot is in the capture pose; take a photo by running `python src/camera_api.py`. Rename/move this newly captured image to the path specified by var CAPTURE_PATH from const.py.

8. Run `python src/main.py` to capture a fresh image and trigger the recalibrate sequence. For the first calibration pop up, select the four points of the puzzle scatter workspace in clockwise order starting in the top left hand corner. For the second calibration pop up, select the calibration points in the order that they are stored in the list CALIBRATION_ROBOT_POINTS. 

9. Set RECALIBRATE=0 in const.py.
   
10. Set SCATTER_AREA_PX = (0, 0, x, y) where (x, y) is directly from calibration/config.json "output_size".

11. Set PUZZLE_SOLVED_SIZE_M_ACTUAL = (width, length) of the puzzle in m.

12. Set PUZZLE_SOLVED_SIZE_M = (width + eps, length + eps) where eps is a small 0.1-0.5 value of gap between pieces. 

13. Done.