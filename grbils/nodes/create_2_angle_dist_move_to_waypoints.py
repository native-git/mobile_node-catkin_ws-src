#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist, PoseStamped
from math import pow,atan2,sqrt,floor
from nav_msgs.msg import Odometry, Path
import tf
from math import radians, degrees, sqrt
from time import sleep
from ca_msgs.msg import Bumper
from roscpp_tutorials.srv import TwoInts

points_traveled = []

rospy.init_node("create_2_angle_dist_move_to_waypoints")
path_publisher = rospy.Publisher('roomba3/location', Path, queue_size=10)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()

def rotate_client(theta):
	rospy.wait_for_service('roomba3/rotate')
	try:
		rotate = rospy.ServiceProxy('roomba3/rotate', TwoInts)
		resp = rotate(theta,0)
		print "Error in rotate: " + str(resp.sum)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def move_client(x):
	rospy.wait_for_service('roomba3/move')
	try:
		move = rospy.ServiceProxy('roomba3/move', TwoInts)
		resp = move(x,0)
		print "Error in move: "+str(resp.sum)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def publish_plan():
	global points_traveled
	taken_path = Path()
	taken_path.header.frame_id = 'map'
	for point in points_traveled:
		way_point = PoseStamped()
		way_point.pose.position.x = point[0]
		way_point.pose.position.y = point[1]
		taken_path.poses.append(way_point)
	path_publisher.publish(taken_path)

def update_path():
	global points_traveled
	(trans,rot) = check_camera()
	x = trans[0]
	y = trans[1]
	points_traveled.append([x,y])
	publish_plan()

def get_plan():

	new_plan = []

	plan = rospy.wait_for_message('roomba3/waypoint', Path)
	print "Calculating a new plan..."
	for point in plan.poses:
		x = point.pose.position.x
		y = point.pose.position.y
		new_point = [x,y]
		new_plan.append(new_point)
	return new_plan

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

def rotateTo(theta):
	facing_target = False

	sleep(2)

	while not facing_target:
		(t1,r1) = check_camera()
		rot = tf.transformations.euler_from_quaternion(r1)
		yaw = rot[2]
		d_theta = theta - yaw
		d_theta = int(round(degrees(d_theta)))
		if d_theta == 360:
			d_theta = 0
		if abs(d_theta) > 180 and d_theta < 0:
			d_theta += 360
		if abs(d_theta) > 180 and d_theta > 0:
			d_theta -= 360
		if abs(d_theta) <= 2:
			facing_target = True
		else:
			print "Rotating to: " + str(d_theta)
			rotate_client(d_theta)
			sleep(2)

def moveTo(distance):

	sleep(2)
	target_achieved = False

	target = distance 
	(trans,rot) = check_camera()
	target *= 1000.0
	print "Moving to: " +str(int(floor(target)))
	move_client(int(floor(target)))
	"""
	x_init = trans[0]
	y_init = trans[1]

	if target >= 0:
		sign = 1
	if target < 0:
		sign = -1

	while not target_achieved:

		(trans,quat) = check_camera()
		cur_x = trans[0]
		cur_y = trans[1]
		cur_d = sqrt((cur_x - x_init)**2 + (cur_y - y_init)**2)
		error = abs(target) - cur_d
		error *= 1000
		if abs(error) <= 10:
			target_achieved = True
		else:
			print "Moving to: " +str(int(floor(error)))
			move_client(int(floor(error)))
			sleep(3)
	"""

def moveBreak(target_x,target_y):

	(trans, rot) = check_camera()
	x_init = trans[0]
	y_init = trans[1]
	
	points = []

	total_x = target_x-x_init
	total_y = target_y-y_init
	distance = sqrt((total_x)**2+(total_y)**2)
	step = 1
	d = step

	while d < distance:

		step_x = (d*total_x)/distance
		step_y = (d*total_y)/distance
		x = x_init + step_x
		y = y_init + step_y
		points.append([x,y])
		d += step

	points.append([target_x,target_y])
	print points
	return points

def move2goal(way_point):

	(trans, rot) = check_camera()
	current_x = trans[0]
	current_y = trans[1]

	target_x = way_point[0]
	target_y = way_point[1]
	points = moveBreak(target_x,target_y)
	update_path()
	for point in points:
		point_x = point[0]
		point_y = point[1]
		(trans, rot) = check_camera()
		current_x = trans[0]
		current_y = trans[1]
		direction = atan2((point_y-current_y),(point_x-current_x))
		print degrees(direction)
		rotateTo(direction)
		sleep(2)
		(trans, rot) = check_camera()
		current_x = trans[0]
		current_y = trans[1]
		movement_distance = sqrt((point_x-current_x)**2+(point_y-current_y)**2)
		moveTo(movement_distance)
		sleep(1)
		update_path()

while not rospy.is_shutdown():
	#global points_traveled
	#print check_camera()
	new_plan = get_plan()
	update_path()
	for way_point in new_plan:
		rospy.set_param('target_achieved', False)
		move2goal(way_point)
		rospy.set_param('target_achieved', True)
	update_path()
	#del points_traveled[:]