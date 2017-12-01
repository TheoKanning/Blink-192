import RPi.GPIO as GPIO

DR = 16
DL = 19

NO_OBSTACLE = 0
LEFT_OBSTACLE = 1
RIGHT_OBSTACLE = 2
FRONT_OBSTACLE = 3 # obstacle on both right and left


"""
Basic driver for HC-SRO4 ultrasonic distance finder
"""
class ObstacleDetector:
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
        GPIO.setup(DL,GPIO.IN,GPIO.PUD_UP)
        
    def detect(self):
        """
        Return flag indicating if there is an obstacle in the way
        """
        DR_status = GPIO.input(DR)
	DL_status = GPIO.input(DL)
	if((DL_status == 1) and (DR_status == 1)):
	    return NO_OBSTACLE
	elif((DL_status == 1) and (DR_status == 0)):
            return RIGHT_OBSTACLE
	elif((DL_status == 0) and (DR_status == 1)):
	    return LEFT_OBSTACLE
	else:
            return FRONT_OBSTACLE
    
