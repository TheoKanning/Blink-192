import cv2
import argparse
from ball_tracker import BallTracker

parser = argparse.ArgumentParser("Test object recognition")
parser.add_argument('--low', nargs=3, type=int, help='RGB values for lower detection threshold', default=(15, 86, 80))
parser.add_argument('--high', nargs=3, type=int, help='RGB values for upper detection threshold', default=(40, 255, 255))
parser.add_argument('--show-mask', nargs='?', type=bool, help='Add this option to show the mask frames', const=True, default=False)
args = parser.parse_args()

tracker = BallTracker(lower_threshold=tuple(args.low), upper_threshold=tuple(args.high), show_debug_windows=args.show_mask)

# keep looping
while True:

    tracker.get_position()

    # if the 'q' key is pressed, stop the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

tracker.release()
