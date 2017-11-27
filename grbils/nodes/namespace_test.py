#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

rospy.init_node('namespace_test')

empty_publisher = rospy.Publisher('test', Empty, queue_size=10)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
	msg = Empty()
	empty_publisher.publish(msg)
	rate.sleep()