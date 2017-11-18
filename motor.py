from AlphaBot import AlphaBot

P = 0.1
I = 0
D = 0.01


class Motor:
    """
    Wrapper around the alphabot motor code
    """

    def __init__(self):
        self.bot = AlphaBot()
        self.bot.stop()
        self.max_speed = 35  # 35% duty cycle
        self.last_proportional = 0  # last proportional error, used by PID control
        self.integral = 0

    def steer(self, angle):
        """
        Moves forward and changes direction by the desired angle. Uses internal
        PID control to make smooth turn.
        @param angle: Desired angular change in radians.
        """
        self.forward()

        proportional = angle
        derivative = proportional - self.last_proportional
        self.last_proportional = proportional

        self.integral += proportional

        # positive -> turn right, negative -> turn left
        differential = P * proportional + I * self.integral + D * derivative
        differential = max(-self.max_speed, min(self.max_speed, differential))  # clamp between +/- max

        print("Motor - error=%f radians, differential=%f" % (proportional, differential))

        if differential < 0:
            self.bot.setPWMB(self.max_speed + differential)
            self.bot.setPWMA(self.max_speed)
        else:
            self.bot.setPWMB(self.max_speed)
            self.bot.setPWMA(self.max_speed - differential)

    def stop(self):
        print("Motor - Stopping")
        self.bot.stop()
        self.__reset_differential()

    def forward(self):
        print("Motor - Forward")
        self.bot.forward()
        self.__reset_differential()

    def backward(self):
        print("Motor - Backward")
        self.bot.backward()
        self.__reset_differential()

    def right(self):
        """
        Rotate to the right while staying in place
        """
        print("Motor - Right")
        self.bot.right()
        self.__reset_differential()

    def left(self):
        """
        Rotate to the left while staying in place
        """
        print("Motor - Left")
        self.bot.left()
        self.__reset_differential()

    def set_speed(self, speed):
        """
        Sets the current maximum duty cycle. Will interrupt any differential steering.
        @param speed: Duty cycle from 0-100
        """
        if speed > 100 or speed < 0:
            raise ValueError("Speed must be between 0 and 100. Speed=%d" % (speed))

        self.max_speed = speed
        self.bot.setPWMA(speed)
        self.bot.setPWMB(speed)

    def __reset_differential(self):
        """
        Resets left and right duty cycles. Will interrupt differential steering.
        """
        self.set_speed(self.max_speed)
