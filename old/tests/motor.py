import argparse
from blink192.motor import Motor

parser = argparse.ArgumentParser(description='Test motor steering')
parser.add_argument('--angle', type=float, help='Desired steering angle in degrees', default=0)
args = parser.parse_args()

motor = Motor()
motor.steer(args.angle) 

raw_input("Press enter to cancel")
