import argparse
import timeit
import cv2

from blink192.tracker import BallTracker
from blink192.tracker import yellowLower
from blink192.tracker import yellowUpper

parser = argparse.ArgumentParser("Test object recognition")
parser.add_argument('--low', nargs=3, type=int, help='HSV values for lower detection threshold', default=yellowLower)
parser.add_argument('--high', nargs=3, type=int, help='HSV values for upper detection threshold', default=yellowUpper)
parser.add_argument('--debug', nargs='?', type=bool, help='Add this option to show the mask frames', const=True, default=False)
args = parser.parse_args()

tracker = BallTracker(lower_threshold=tuple(args.low), upper_threshold=tuple(args.high), show_debug_windows=args.debug)

while True:
    
    time = timeit.timeit(tracker.get_position, number=1)
    print("Took %f ms" % (time))

    # if the 'q' key is pressed, stop the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

tracker.release()
