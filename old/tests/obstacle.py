import blink192.obstacle as obstacle
import time

detector = obstacle.ObstacleDetector()

while True:
    result = detector.detect()
    if result == obstacle.NO_OBSTACLE:
        print("No Obstacles")
    elif result == obstacle.RIGHT_OBSTACLE:
        print("Right oBstacle")
    elif result == obstacle.LEFT_OBSTACLE:
        print("Left Obstacle")
    else:
        print("Left and Right Obstacles")
    time.sleep(0.25)
