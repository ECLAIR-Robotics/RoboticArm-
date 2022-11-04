import LeArm
import Camera
import time
import cv2 as cv

arm = LeArm.Controller(port="/dev/cu.usbmodem11201", simulation=False)

arm.home(action_time=2000)

arm.logActions()

