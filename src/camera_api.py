import depthai as dai
import cv2
from const import *

'''
this file contains black box functions for main.py by wrapping api specific functions to output generic types.
this way, if you change the camera in the future all you need to edit is this file and strictly nothing else
uses depth api to capture with luxonis cam
'''

class Camera:
    def __init__(self,  simulated=SIMULATION):
        self.simulated=simulated

    def capture(self, image_path = "media/captures/Capture.png"):
        if not self.simulated:
            # Connect to device and create pipeline
            with dai.Pipeline() as pipeline:
                # OAK-D Pro's color camera is on socket CAM_A (center)
                camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

                # Request a still/video output at a given resolution
                output = camRgb.requestOutput((1920, 1080))
                videoQueue = output.createOutputQueue()

                pipeline.start()

                # Grab one frame
                videoFrame = videoQueue.get()
                frame = videoFrame.getCvFrame()

                # Save it
                cv2.imwrite(image_path, frame) 
                print(f"Saved {image_path}") # save image 
        else:
            print("logging: camera_api SIMULATED RUN")
