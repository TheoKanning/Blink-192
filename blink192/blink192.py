import motor
import tracker
import obstacle
import collector

tracker = tracker.BallTracker()
detector = obstacle.ObstacleDetector()
motor = motor.Motor()
collector = collector.ContinuousCollector()

collector.start()

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
    # convert ball_x into releative angle in degrees
    # camera has 640 pixels in ~60 degrees of vision
    angle = - (ball_x - 320) * 30 / 320
    motor.steer(angle)
    

