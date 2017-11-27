#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int16

rospy.init_node("create_2_bumper_stop")

mode_publisher = rospy.Publisher("mode", Int16, queue_size=10)

def bumper_callback(msg):
	mode = Int16()
	if msg.data == True:
		mode.data = 1
		mode_publisher.publish(mode)

def wheel_drop_callback(msg):
	mode = Int16()
	if msg.data == True:
		mode.data = 1
		mode_publisher.publish(mode)

def subscribers():
	rospy.Subscriber("bumper", Bool, bumper_callback)
	rospy.Subscriber("wheel_drop", Bool, wheel_drop_callback)
	rospy.spin()

subscribers()