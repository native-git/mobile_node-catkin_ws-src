#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import time
import threading
import numpy as np

rospy.init_node('odom_static_tf_broadcaster')
rate = rospy.Rate(15)
listener = tf.TransformListener()
tfBuffer = tf2_ros.Buffer()
listener2 = tf2_ros.TransformListener(tfBuffer)

t = tf.Transformer(True, rospy.Duration(20.0))
broadcaster = tf2_ros.StaticTransformBroadcaster()

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
			(trans,rot) = listener.lookupTransform("/alfred/base_link","/alfred/odom", rospy.Time(0))
			got_one = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rate.sleep()
		return (trans, rot)

def lookup_camera():
	got_one = False
	while not got_one:
		
		try:
			(trans,rot) = listener.lookupTransform("/map", "/alfred", rospy.Time(0))
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
			(trans,quat) = listener.lookupTransform('/map', '/alfred', rospy.Time(0))
			new_trans = np.array(trans)
			v_vect = new_trans - old_trans
			#print v_vect
			old_trans = new_trans
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "failed"
			#continue
		v_vect = list(v_vect)
		if any(abs(v)>=0.75 for v in v_vect):
			stable = False
		else:
			stable = True
		if any(abs(v)>=0.01 for v in v_vect):
			stationary = False
		else:
			stationary = True
		rate.sleep()

def check_error():
	global is_accurate
	while not rospy.is_shutdown():
		try:
			print tfBuffer.lookup_transform('alfred', 'alfred/base_link', rospy.Time(0))
			print listener.lookupTransform('alfred','alfred/base_link',rospy.Time(0))
			#(trans,quat) = tfBuffer.lookup_transform('alfred', 'alfred/base_link', rospy.Time(0))
			#trans = (trans[0],trans[1],0.0)
			#euler = tf.transformations.euler_from_quaternion(quat)
			#yaw = math.degrees(euler[2])
			#print trans,yaw
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print "failed"
			#continue
		"""
		if any(abs(e)>=0.01 for e in trans) or abs(yaw)>=2:
			is_accurate = False
		else:
			is_accurate = True
		"""
		rate.sleep()

def broadcast_static((trans,rot)):

	static_transformStamped = geometry_msgs.msg.TransformStamped()
	static_transformStamped.header.stamp = rospy.Time.now()
	static_transformStamped.header.frame_id = "map"
	static_transformStamped.child_frame_id = "alfred/odom"

	static_transformStamped.transform.translation.x = trans[0]
	static_transformStamped.transform.translation.y = trans[1]

	static_transformStamped.transform.rotation.z = rot[2]
	static_transformStamped.transform.rotation.w = rot[3]

	broadcaster.sendTransform(static_transformStamped)

def update_odom_static():
	global is_accurate
	set_transform('map_temp','camera',lookup_camera())
	set_transform('camera','odom_temp',lookup_odom())
	broadcast_static(get_transform('map_temp','odom_temp'))
	is_accurate = True

camera_thread = threading.Thread(target=check_camera)
camera_thread.daemon = True
camera_thread.start()

error_thread = threading.Thread(target=check_error)
error_thread.daemon = True
error_thread.start()

update_odom_static()

while camera_thread.is_alive():
	#print "Stationary: ",stationary," Stable: ",stable," Is_accurate: ",is_accurate
	if stationary and stable and is_accurate == False:
		update_odom_static()
		print "updating odom tf"
	time.sleep(0.5)