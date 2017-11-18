from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import time

buffer_length = 16
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)


class BallTracker:
    def __init__(self, lower_threshold=greenLower, upper_threshold=greenUpper, show_debug_windows=False):
        self.pts = deque(maxlen=buffer_length)
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rgbArray = PiRGBArray(self.camera, size=(640, 480))

        self.lower_threshold = lower_threshold
        self.upper_threshold = upper_threshold
        self.show_debug_windows=show_debug_windows

        # allow camera to warm up
        time.sleep(0.1)

    def get_position(self):
        """
        Takes a picture and returns the location of the first ball found. Returns None
        if ball can't be found
        returns: (x, y) coordinates of ball, or None
        """
        self.camera.capture(self.rgbArray, format="bgr", use_video_port=True)

        image = self.rgbArray.array
        self.rgbArray.truncate(0)

        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # get pixels matching threshold colors
        mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        if self.show_debug_windows:
            cv2.imshow("Mask", mask)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image, center, 5, (0, 0, 255), -1)

        # update the points queue
        self.pts.appendleft(center)

        # loop over the set of tracked points
        for i in range(1, len(self.pts)):
            # if either of the tracked points are None, ignore
            # them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(buffer_length / float(i + 1)) * 2.5)
            cv2.line(image, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

        # show the frame to our screen
        cv2.imshow("Frame", image)

        return center

    def release(self):
        """
        Release all resources. Can not be used after this is called.
        """
        self.camera.close()
        cv2.destroyAllWindows()
