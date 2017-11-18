import cv2
import argparse
from ball_tracker import BallTracker

parser = argparse.ArgumentParser("Test object recognition")
parser.add_argument('--low', nargs='3', type=int, help='RGB values for lower detection threshold', default=(29, 86, 6))
parser.add_argument('--high', nargs='3', type=int, help='RGB values for upper detection threshold', default=(64, 255, 255))
args = parser.parse_args()

tracker = BallTracker(lower_threshold=args.low, upper_threshold=args.high, show_debug_windows=True)

# keep looping
while True:

    tracker.get_position()

    # if the 'q' key is pressed, stop the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

tracker.release()
