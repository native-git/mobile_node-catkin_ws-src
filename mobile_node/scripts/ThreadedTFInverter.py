#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
from threading import Thread


def sendTF(inv_trans, inv_rot, fromName, toName):
	br = tf.TransformBroadcaster()

	## traslation, rotation, time of publish, to-frame, from-frame
	br.sendTransform( (inv_trans[0], inv_trans[1], inv_trans[2]),
		(inv_rot[0], inv_rot[1], inv_rot[2], inv_rot[3]), 
		rospy.Time.now(),
		toName,
		fromName)

def usb_cam():
	while not rospy.is_shutdown():
		#print "inside usb_cam"
		try:
			## lookup - from-frame, to-frame, time of transform
			(trans,rot) = listener.lookupTransform('/usb_cam2/ceiling_grid', '/usb_cam2', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		
		print "inside usb_cam"
		## translation, rotation, from-frame, to-frame
		sendTF(trans, rot, '/ceiling_grid', '/usb_cam2_new')



def hd_cam():
	
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/hd_cam/ceiling_grid', '/hd_cam', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		print "inside hd_cam"
		sendTF(trans, rot, '/ceiling_grid', '/hd_cam_new')


def Main():
	hd_cam_thread = Thread(target=hd_cam)
	hd_cam_thread.start()

	usb_cam_thread = Thread(target=usb_cam)
	usb_cam_thread.start()


					
if __name__ == '__main__':
	rospy.init_node('tf_inverter')
	listener = tf.TransformListener()
	rate = rospy.Rate(10)
	Main()
