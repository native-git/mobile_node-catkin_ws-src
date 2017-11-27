#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tf
from math import radians, degrees
from numpy import sign
from time import sleep
import threading

print "Initializing, this may take a moment..."

dist = []

def wait_for_key():

	x = raw_input("Press enter when you want to record position...")

def test_func():

	while thread.is_alive():
		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = euler[2]
		print "Current angle: " + str(degrees(yaw))
		print "----"
		rate.sleep()
		pass

	(trans,quat) = check_camera()
	euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	starting_angle = degrees(yaw)
	print "Starting angle: " + str(starting_angle)

	thread2 = threading.Thread(target=wait_for_key)
	thread2.daemon = True
	thread2.start()

	while thread2.is_alive():
		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = euler[2]
		print "Starting angle: " + str(starting_angle)
		print "Current angle: " + str(degrees(yaw))
		print "---"
		rate.sleep()
		pass

	(trans,quat) = check_camera()
	euler = tf.transformations.euler_from_quaternion(quat)
	yaw = euler[2]
	final_angle = degrees(yaw)
	dif = abs(final_angle - starting_angle)
	print "Final angle: " + str(final_angle)
	print "Starting angle: " + str(starting_angle)
	print "Difference: " + str(dif)
	dist.append(dif)
	print dist

	print("Thread is over.")

rospy.init_node("roomba_pid_controller")

rate = rospy.Rate(10.0)

listener = tf.TransformListener()

def check_camera():
	
	got_one = False

	while not got_one:

		try:
			(t,rot) = listener.lookupTransform('/map', '/roomba3', rospy.Time(0))
			got_one = True

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[2]
	quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
	trans = (t[0],t[1],0.0)
	return (trans,quat)

for i in range(10):
	thread = threading.Thread(target=wait_for_key)
	thread.daemon = True
	thread.start()
	test_func()

total = 0

print dist

for d in dist:
	total += d
avg = total/float(len(dist))
print "Average Distance: " + str(avg)
