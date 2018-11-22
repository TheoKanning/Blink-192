#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class VisionNode():
    def __init__(self):
        self.pub = rospy.Publisher('ball_position', String, queue_size=10)
        self.sub = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, self.callback)

    def callback(self, image):
        message = "Image received %s" % rospy.get_time()
        rospy.loginfo(message)
        self.pub.publish(message)

if __name__ == '__main__':
    try:
        rospy.init_node('vision', anonymous=True)
        vision = VisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
