#!/usr/bin/env python

import rospy
import math
import tf
from time import sleep
from geometry_msgs.msg import Twist

def Main():

	while not rospy.is_shutdown():

		try:
			(trans,rot) = listener.lookupTransform('/map_zero', '/map', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		# trans contains the x, y, and z components of the position of the camera with respect to map (units are in meters)

		# rot is a quaternion containing the rotational components of the translation between the map and the camera
		# Since euler angles are somewhat easier to work with, we will convert to those:
		
		# Lastly, since the default units are radians, we will convert to degrees since it is more intuitive

		print trans
		rate.sleep()

if __name__ == '__main__':
	rospy.init_node('position_publisher')
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	Main()
