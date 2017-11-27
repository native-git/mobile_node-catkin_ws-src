#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time
import threading
import numpy as np
from math import degrees, radians

rospy.init_node('tf_to_odom_publisher')
rate = rospy.Rate(15)

listener = tf.TransformListener()

odom_publisher = rospy.Publisher("roomba3/odom", Odometry, queue_size=10)

def check_camera():
	x_old = 0.0
	y_old = 0.0
	z_old = 0.0
	old_trans = np.array([x_old,y_old,z_old])
	old_time = time.time()
	yaw_old = 0.0
	while not rospy.is_shutdown():

		#msg = PoseWithCovarianceStamped()
		msg = Odometry()

		try:
			(trans,quat) = listener.lookupTransform('roomba3/odom', 'roomba3/base_link', rospy.Time(0))
			new_time = time.time()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = 'roomba3/odom'
			msg.child_frame_id = 'roomba3/base_link'
			euler = tf.transformations.euler_from_quaternion(quat)
			roll = degrees(euler[0])
			pitch = degrees(euler[1])
			yaw = degrees(euler[2])
			new_trans = np.array(trans)
			v_ang = radians(yaw-yaw_old)
			v_vect = new_trans - old_trans
			v_vect *= 1.0/(new_time-old_time)
			v_ang *= 1.0/(new_time-old_time)
			old_trans = new_trans
			old_time = new_time
			yaw_old = yaw
			v_vect = list(v_vect)

			for i in range(len(v_vect)):
				msg.pose.covariance[i*7]=0.01 # covariances for [x,y,z]
				#msg.twist.covariance[i*7]=0.001
				msg.pose.covariance[(i*7)+21]=0.008 # covariances for [roll,pitch,yaw]
				msg.twist.covariance[(i*7)+21]=0.008

			msg.twist.covariance[0] = 0.01
			msg.pose.pose.position.x = trans[0]
			msg.pose.pose.position.y = trans[1]
			msg.pose.pose.position.z = 0.0

			msg.pose.pose.orientation.x = 0.0
			msg.pose.pose.orientation.y = 0.0
			msg.pose.pose.orientation.z = quat[2]
			msg.pose.pose.orientation.w = quat[3]

			msg.twist.twist.linear.x = v_vect[0]
			msg.twist.twist.linear.y = 0.0
			msg.twist.twist.linear.z = 0.0

			msg.twist.twist.angular.z = v_ang

			odom_publisher.publish(msg)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "failed"
			#continue
		rate.sleep()

check_camera()