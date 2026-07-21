import depthai as dai
import cv2
from const import *

from datetime import datetime

'''
this file contains black box functions for main.py by wrapping api specific functions to output generic types.
this way, if you change the camera in the future all you need to edit is this file and strictly nothing else
uses depth api to capture with luxonis cam


'''

class Camera:
    def __init__(self,  camera_type =CAMERA_TYPE, simulated=SIMULATION):
        if CAMERA_TYPE == DEPTHAI:
            self.camera=DepthAICamera(simulated=SIMULATION)

    def capture(self, image_path = f"media/captures/Capture_{datetime.now()}.png"):
        self.camera.capture(image_path=image_path)


class DepthAICamera:
    def __init__(self, simulated=SIMULATION):
        self.simulated = simulated
    
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


if __name__ == "__main__":
    c = Camera(simulated=0)
    c.capture(f"testphoto_{datetime.now()}.png")