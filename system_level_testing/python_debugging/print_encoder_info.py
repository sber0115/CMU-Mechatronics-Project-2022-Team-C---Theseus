#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

def callback(data)
	rospy.loginfo("I heard %" data)


rospy.init_node('listener', anonymous=True)
rospy.Subscriber("encoder_info", Int64, callback)
rospy.spin()