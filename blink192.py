import ball_tracker as tracker
import motor

tracker = tracker.BallTracker()
motor = motor.Motor()

# loop forever
while True:
    # todo this is still in pixels
    coordinates = tracker.get_position()

    if coordinates is None:
        # nothing to do until we see a ball
        motor.forward()
        continue

    ball_x, ball_y = coordinates
    # convert ball_x into something closer to radians
    # camera has 640 pixels in ~90 degrees of vision
    # todo use real math based on angles and perceived distance
    ball_x = (ball_x - 320) * 1.57 / 640
    motor.steer(ball_x)
