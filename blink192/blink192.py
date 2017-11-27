from blink192 import motor
from blink192 import tracker as tracker

tracker = tracker.BallTracker()
motor = motor.Motor()

# loop forever
while True:
    # todo this is still in pixels
    coordinates = tracker.get_position()

    if coordinates is None:
        # nothing to do until we see a ball
        print("No ball found")
        motor.stop()
        continue

    ball_x, ball_y = coordinates
    # convert ball_x into something closer to radians
    # camera has 640 pixels in ~90 degrees of vision
    # todo use real math based on angles and perceived distance
    ball_x = - (ball_x - 320) * 1.57 / 640
    motor.steer(ball_x)
    

