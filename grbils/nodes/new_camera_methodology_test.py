#! /usr/bin/env python

import random
import rospy
import tf
from roscpp_tutorials.srv import TwoInts
import os
import move_base_msgs.msg
import actionlib
import geometry_msgs.msg
import time
import csv
from std_srvs.srv import Empty
import math

rospy.init_node("new_camera_methodology_test")
rate = rospy.Rate(10.0)

listener = tf.TransformListener()
pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size = 1)

t = tf.Transformer(True, rospy.Duration(20.0))

def request_update():
	rospy.set_param("/request_update", True)

def get_transform(from_a,to_b):

	(trans,rot) = t.lookupTransform(from_a,to_b,rospy.Time(0))
	return (trans,rot)

def kill_odom_transform():
	os.system("rosnode kill /map_2_odom >/dev/null &")

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

def launch_static(x,y,qz,qw):
	command = "roslaunch grbils map_2_odom.launch "
	command += "x:=" +str(x) + " y:=" +str(y) + " qx:=0.0 qy:=0.0 qz:=" + str(qz) + " qw:=" + str(qw) +" >/dev/null &"
	print command
	os.system(command)

def initialize():
	check_camera()
	(trans,rot) = get_transform("map","pioneer")
	set_transform("map","origin",(trans,rot))
	launch_static(trans[0],trans[1],rot[2],rot[3])

def check_camera():

	request_update()
	request_pending = True
	while request_pending:
		camera_updated = rospy.get_param("/camera_updated", False)
		if camera_updated:
			rospy.set_param("/camera_updated", False)
			print "Camera Updated"
			request_pending = False
			break

	i = 0
	x_old = 0
	y_old = 0
	yaw_old = 0

	got_position = False

	while not got_position:

		rate.sleep()

		try:
			(trans,rot) = listener.lookupTransform('/map', '/pioneer', rospy.Time(0))
			rate.sleep()
			i += 1
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		euler = tf.transformations.euler_from_quaternion(rot)
		x = trans[0]
		y = trans[1]
		yaw = euler[2]
		
		if (x == x_old) and (y == y_old) and (yaw == yaw_old):
			print "Position has converged"
			quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
			trans = (x,y,0.0)
			set_transform("map","pioneer",(trans,quaternion))
			print "X: " + str(x)
			print "Y: " + str(y)
			print "YAW: " + str(yaw)
			got_position = True
		else:
			x_old = x
			y_old = y
			yaw_old = yaw

def log_and_print(point_number,point_type,(trans,rot)):
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = math.degrees(euler[2])
	writer.writerow({'point_number': point_number, 'point_type': point_type, 'x': trans[0], 'y': trans[1], 'yaw': yaw})
	print 'point_number:',point_number,'point_type:',point_type,'x:',trans[0],'y:',trans[1],'yaw:',yaw,""

#with open('test_8_camera_and_odom_return_to_zero_with_approach_modified_tolerance_and_costmap_resolution.csv', 'ab') as csvfile:
with open('test.csv', 'ab') as csvfile:
	
	fieldnames = ['point_number','point_type','x','y','yaw']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	kill_odom_transform()
	initialize()
	log_and_print(0,'initial_point',get_transform("map","origin"))

	for i in range(1,21):

		raw_input("Press enter to continue...")
		check_camera()
		log_and_print(i,'camera_position_with_sleep',get_transform("map","pioneer"))