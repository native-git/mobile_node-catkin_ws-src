#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import threading
import numpy as np
from math import degrees, radians

rospy.init_node('tf_to_pose_publisher')
rate = rospy.Rate(15)

listener = tf.TransformListener()

pose_publisher = rospy.Publisher("roomba3/pose", PoseWithCovarianceStamped, queue_size=10)

def check_camera():
	x_old = 0.0
	y_old = 0.0
	z_old = 0.0
	old_trans = np.array([x_old,y_old,z_old])
	old_time = time.time()
	yaw_old = 0.0
	while not rospy.is_shutdown():

		msg = PoseWithCovarianceStamped()

		try:
			(trans,quat) = listener.lookupTransform('/map', '/roomba3', rospy.Time(0))
			new_time = time.time()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = 'map'
			euler = tf.transformations.euler_from_quaternion(quat)
			roll = degrees(euler[0])
			pitch = degrees(euler[1])
			yaw = degrees(euler[2])
			new_trans = np.array(trans)
			v_vect = new_trans - old_trans
			v_vect *= 1.0/(new_time-old_time)
			v_ang = radians(abs(yaw-yaw_old))/(new_time-old_time)
			#print v_ang
			yaw_old = yaw
			old_trans = new_trans
			old_time = new_time
			v_vect = list(v_vect)
			inflate = False
			for i in range(len(v_vect)):
				msg.pose.covariance[i*7]=max(0.001,abs(v_vect[i]))+v_ang # covariances for [x,y,z]
				#msg.pose.covariance[i*7]=max(0.001,abs(v_vect[i]))
				msg.pose.covariance[(i*7)+21]=0.008 # covariances for [roll,pitch,yaw] (orientation usually accurate to ~0.5 deg)
			if (abs(roll)>=10 or abs(pitch)>=10):
				# if orientation data does not make sense, inflate covariances for position
				inflate = True
			if (trans[0]<=-16 or trans[0]>=2 or trans[1]<=4 or trans[1]>=20 or abs(trans[2]+0.1)>=0.075):
				# if any of the positions appear to be outside of the valid range, inflate covariances
				inflate = True
			if inflate:
				for i in range(len(v_vect)):
					msg.pose.covariance[i*7] = 5
			
			#msg.pose.covariance[5]=max(0.001,v_vect[0])*max(1,v_ang)

			msg.pose.pose.position.x = trans[0]
			msg.pose.pose.position.y = trans[1]
			msg.pose.pose.position.z = 0.0

			msg.pose.pose.orientation.x = 0.0
			msg.pose.pose.orientation.y = 0.0
			msg.pose.pose.orientation.z = quat[2]
			msg.pose.pose.orientation.w = quat[3]

			pose_publisher.publish(msg)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "failed"
			#continue
		rate.sleep()

check_camera()