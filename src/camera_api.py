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
            self.camera=DepthAICamera(simulated=simulated)

    def capture(self, image_path = f"media/captures/Capture_{datetime.now()}.png"):
        self.camera.capture(image_path=image_path)


class DepthAICamera:
    def __init__(self, simulated=SIMULATION):
        self.simulated = simulated
        print("Init DAI")
        print(f"sim: {self.simulated}")
    
    def capture(self, image_path="media/captures/Capture.png"):
        if not self.simulated:
            with dai.Pipeline() as pipeline:
                camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
                output = camRgb.requestOutput((1920, 1080))
                videoQueue = output.createOutputQueue()
                controlQueue = camRgb.inputControl.createInputQueue()

                pipeline.start()

                ctrl = dai.CameraControl()
                ctrl.setManualExposure(exposureTimeUs=5000, sensitivityIso=200)
                ctrl.setManualFocus(lensPosition=130)
                ctrl.setSharpness(0)
                ctrl.setLumaDenoise(0)
                ctrl.setChromaDenoise(4)
                controlQueue.send(ctrl)

                import time
                time.sleep(3.0)

                videoFrame = videoQueue.get()
                frame = videoFrame.getCvFrame()

                cv2.imwrite(image_path, frame)
                print(f"Saved {image_path}")
        else:
            print("logging: camera_api SIMULATED RUN")

if __name__ == "__main__":
    c = Camera(simulated=0)
    c.capture(f"testphoto_{datetime.now()}.png")