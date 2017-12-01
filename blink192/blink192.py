import motor
import tracker
import obstacle

tracker = tracker.BallTracker()
detector = obstacle.ObstacleDetector()
motor = motor.Motor()

# loop forever
while True:
    
    obstacle_result = detector.detect()
    if obstacle_result == obstacle.LEFT_OBSTACLE:
        print("Obstacle detected, turning right")
        motor.right()
        continue
    elif obstacle_result == obstacle.RIGHT_OBSTACLE:
        print("Obstacle detected, turning left")
        motor.left()
        continue
    elif obstacle_result == obstacle.FRONT_OBSTACLE:
        print("Obstacle detected, moving back")
        motor.backward()
        continue

    # todo this is still in pixels
    coordinates = tracker.get_position()

    if coordinates is None:
        # nothing to do until we see a ball
        print("No ball found, moving forward")
        motor.forward()
        continue

    ball_x, ball_y = coordinates
    # convert ball_x into something closer to radians
    # camera has 640 pixels in ~90 degrees of vision
    # todo use real math based on angles and perceived distance
    ball_x = - (ball_x - 320) * 1.57 / 640
    motor.steer(ball_x)
    

