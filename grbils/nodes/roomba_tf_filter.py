#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import time
import threading
import numpy as np
from math import degrees

rospy.init_node('roomba_tf_filter')
rate = rospy.Rate(15)

listener = tf.TransformListener()

t = tf.Transformer(True, rospy.Duration(20.0))

time.sleep(1)

stable = False
stationary = False
is_accurate = False

def get_transform(from_a,to_b):
	(trans,rot) = t.lookupTransform(from_a,to_b,rospy.Time(0))
	return (trans,rot)

def set_transform(from_a,to_b,(trans,rot)):
	m = geometry_msgs.msg.TransformStamped()
	m.header.frame_id = from_a
	m.child_frame_id = to_b
	m.transform.translation.x = trans[0]
	m.transform.translation.y = trans[1]
	m.transform.rotation.x = 0.0
	m.transform.rotation.y = 0.0
	m.transform.rotation.z = rot[2]
	m.transform.rotation.w = rot[3]
	t.setTransform(m)

def lookup_odom():
	got_one = False
	while not got_one:
		
		try:
			(trans,rot) = listener.lookupTransform("/roomba3/base_link","/roomba3/odom", rospy.Time(0))
			got_one = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rate.sleep()
		return (trans, rot)

def lookup_camera():
	got_one = False
	while not got_one:
		
		try:
			(trans,rot) = listener.lookupTransform("/map", "/roomba3", rospy.Time(0))
			got_one = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rate.sleep()
		trans = (trans[0],trans[1],0.0)
		rot = (0.0,0.0,rot[2],rot[3])
		return (trans, rot)

def check_camera():
	global stationary, stable

	x_old = 0.0
	y_old = 0.0
	z_old = 0.0
	old_trans = np.array([x_old,y_old,z_old])
	while not rospy.is_shutdown():
		try:
			#(trans,quat) = listener.lookupTransform('/alfred/odom', '/alfred/base_link', rospy.Time(0))
			(trans,quat) = listener.lookupTransform('/map', '/roomba3', rospy.Time(0))
			euler = tf.transformations.euler_from_quaternion(quat)
			roll = degrees(euler[0])
			pitch = degrees(euler[1])
			new_trans = np.array(trans)
			v_vect = new_trans - old_trans
			#print v_vect
			old_trans = new_trans
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "failed"
			#continue
		v_vect = list(v_vect)
		c1 = False
		c2 = False
		if (any(abs(v)>=0.75 for v in v_vect) or abs(roll)>=10 or abs(pitch)>=10):
			c1 = True
		if (trans[0]<=-16 or trans[0]>=2 or trans[1]<=4 or trans[1]>=20 or abs(trans[2])>=0.1):
			c2 = True
		if (c1 or c2):
			stable = False
		else:
			stable = True
		if any(abs(v)>=0.002 for v in v_vect):
			stationary = False
		else:
			stationary = True
		rate.sleep()

def check_error():
	global is_accurate, stable, stationary
	while not rospy.is_shutdown():
		try:
			(trans,quat) = listener.lookupTransform('roomba3','roomba3/base_link',rospy.Time(0))
			trans = (trans[0],trans[1],0.0)
			euler = tf.transformations.euler_from_quaternion(quat)
			yaw = degrees(euler[2])
			if (any(abs(e)>=0.005 for e in trans) or abs(yaw)>=0.5) and stable and stationary:
				is_accurate = False
			else:
				is_accurate = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "failed"
			#continue
		rate.sleep()

set_transform('map_temp','camera',lookup_camera())
set_transform('camera','odom_temp',lookup_odom())
(odom_trans,odom_rot) = get_transform('map_temp','odom_temp')

def broadcast_static():
	global odom_trans, odom_rot
	rate2 = rospy.Rate(10)
	while not rospy.is_shutdown():
		br = tf.TransformBroadcaster()
		br.sendTransform(odom_trans,odom_rot,rospy.Time.now(),'roomba3/odom','map')
		rate2.sleep()

def update_odom_static():
	global is_accurate, odom_trans,odom_rot
	set_transform('map_temp','camera',lookup_camera())
	set_transform('camera','odom_temp',lookup_odom())
	(odom_trans,odom_rot) = get_transform('map_temp','odom_temp')
	is_accurate = True

camera_thread = threading.Thread(target=check_camera)
camera_thread.daemon = True
camera_thread.start()

update_odom_static()

static_thread = threading.Thread(target=broadcast_static)
static_thread.daemon = True
static_thread.start()

error_thread = threading.Thread(target=check_error)
error_thread.daemon = True
error_thread.start()

i = 0
while camera_thread.is_alive():
	#print "Stationary: ",stationary," Stable: ",stable," Is_accurate: ",is_accurate
	if stationary and stable:
		i += 1
	else:
		i = 0
	#if stationary and stable and is_accurate == False:
	if i >= 5 and not is_accurate:
		update_odom_static()
		print "updating odom tf"
	time.sleep(0.1)