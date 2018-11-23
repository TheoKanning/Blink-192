#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from alphabot.alphabot import AlphaBot

speed = 50
differential = 50
bot = AlphaBot()

def command_callback(twist):
    left_speed = speed * twist.linear.x - differential * twist.angular.z
    right_speed = speed * twist.linear.x + differential * twist.angular.z
    bot.setMotor(left_speed, right_speed)

if __name__ == '__main__':
    print "Starting motor node"
    rospy.init_node('motors')
    rospy.Subscriber('cmd_vel', Twist, command_callback)
    rospy.spin()
