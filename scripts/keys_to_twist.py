#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {
        'w': [0, 1],
        'x': [0, -1],
        'a': [1, 0],
        'd': [-1, 0],
        's': [0, 0]
    }

twist = Twist()

def keys_callback(msg, twist_pub):
    global twist
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        # unrecognized key
        return

    velocities = key_mapping[msg.data[0]]
    twist.angular.z = velocities[0]
    twist.linear.x = velocities[1]
    twist_pub.publish(twist)

if __name__ == '__main__':
    print "Starting keys_to_twist"
    rospy.init_node('keys_to_twist')
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_callback, twist_pub)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist_pub.publish(twist)
        rate.sleep()
