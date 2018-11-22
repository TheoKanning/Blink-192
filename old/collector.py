import RPi.GPIO as GPIO


SERVO = 27
PWM = 4 # pwm duty cycle, determines rotation direction and spedd

class ContinuousCollector:
    """
    Class that allows continuously collecting small things.
    """
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO, GPIO.OUT, initial=GPIO.LOW)
        self.servo = GPIO.PWM(SERVO,50)
        
    def start(self):
        self.servo.start(PWM)
        
    def stop(self):
        self.servo.stop()
        