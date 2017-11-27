#! /usr/bin/env python

import random
import rospy
import tf
import os
import geometry_msgs.msg
import time
import csv
import math

rospy.init_node("tag_rot_accuracy_test")
rate = rospy.Rate(10.0)

listener = tf.TransformListener()
t = tf.Transformer(True, rospy.Duration(20.0))

def get_transform(from_a,to_b):

	(trans,rot) = t.lookupTransform(from_a,to_b,rospy.Time(0))
	return (trans,rot)

def set_transform(from_a,to_b,trans,rot):

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

def check_camera():

	got_one = False

	while not got_one:
		try:
			(trans,rot) = listener.lookupTransform('/hd_cam_3', 'hd_cam_3/tag_0', rospy.Time(0))
			rate.sleep()
			got_one = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	set_transform("camera","center_of_rotation",trans,rot)

def initialize():
	check_camera()
	(trans,rot) = get_transform("camera","center_of_rotation")
	set_transform("camera","center_of_rotation_starting_point",trans,rot)

def log_and_print(point_number,expected_angle,measured_angle,(trans,rot)):
	euler = tf.transformations.euler_from_quaternion(rot)
	roll = math.degrees(euler[0])
	pitch = math.degrees(euler[1])
	yaw = math.degrees(euler[2])
	writer.writerow({'point_number': point_number, 'expected_angle': expected_angle, 'measured_angle': measured_angle, 'x': trans[0], 'y': trans[1], 'z': trans[2], 'roll': roll, 'pitch': pitch, 'yaw': yaw})
	print 'point_number:',point_number,'expected_angle:',expected_angle,'measured_angle',measured_angle,'x:',trans[0],'y:',trans[1],'z:',trans[2],'roll:',roll,'pitch:',pitch,'yaw:',yaw

with open('tag_rot_accuracy_test6_single_tag_0.5_scale_no_config_default_size_2100.csv', 'ab') as csvfile:

	fieldnames = ['point_number','expected_angle','measured_angle','x','y','z','roll','pitch','yaw']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	initialize()
	log_and_print(0,0.0,0.0,get_transform("camera","center_of_rotation_starting_point"))

	for i in range(0,13):

		raw_input("Press enter to continue...")	
		# Add the camera data to the tf tree
		check_camera()
		(t2,r2) = get_transform("center_of_rotation_starting_point","center_of_rotation")
		euler = tf.transformations.euler_from_quaternion(r2)
		yaw = math.degrees(euler[2])
		expected_angle = i*30
		log_and_print(i,expected_angle,yaw,get_transform("camera","center_of_rotation"))