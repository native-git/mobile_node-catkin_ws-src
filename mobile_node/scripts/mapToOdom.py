#!/usr/bin/env python

import rospy
import math
import tf


def sendTF(trans, rot, fromName, toName):
	br = tf.TransformBroadcaster()

	br.sendTransform( (trans[0], trans[1], trans[2]),
		(rot[0], rot[1], rot[2], rot[3]), 
		rospy.Time.now(),
		toName,
		fromName)

def Main():
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/hd_cam_new', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		print "trans -------------------------"
		print trans

		sendTF(trans, rot, "/map", "/camera")



if __name__ == '__main__':
	rospy.init_node('mapToOdom')
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	Main()