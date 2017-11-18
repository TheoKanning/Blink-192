# import the necessary packages
from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import imutils
import cv2
import time
from ball_tracker import BallTracker

tracker = BallTracker()

# keep looping
while True:

    tracker.get_position()

    # if the 'q' key is pressed, stop the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

tracker.release()
