#! /usr/bin/env python

import rospy
import tf
import csv
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg  import Twist, PoseStamped

points_traveled = []

rospy.init_node("log_position")
path_publisher = rospy.Publisher('/roomba2/location', Path, queue_size=10)
rate = rospy.Rate(10.0)

listener = tf.TransformListener()
t = tf.Transformer(True, rospy.Duration(20.0))

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

def update_path(x,y):
	global points_traveled
	points_traveled.append([x,y])
	publish_plan()

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

def log_and_print(point_number,point_type,(trans,rot)):
	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = math.degrees(euler[2])
	writer.writerow({'point_number': point_number, 'point_type': point_type, 'x': trans[0], 'y': trans[1], 'yaw': yaw})
	print 'point_number:',point_number,'point_type:',point_type,'x:',trans[0],'y:',trans[1],'yaw:',yaw,""

def check_camera():
	
	got_one = False

	while not got_one:

		try:
			(t,rot) = listener.lookupTransform('/map', '/alfred', rospy.Time(0))
			got_one = True

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[2]
	quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
	trans = (t[0],t[1],0.0)
	return (trans,quat)

with open('position_log_4.csv', 'ab') as csvfile:

	fieldnames = ['point_number','point_type','x','y','yaw']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	writer.writeheader()
	i = 0

	while not rospy.is_shutdown():
		test = raw_input("Press enter to write position to .csv file, Enter <q> to quit...")
		if test == 'q':
			break
		else:
			i += 1
			(trans,rot) = check_camera()
			log_and_print(i,"map->robot",(trans,rot))
			update_path(trans[0],trans[1])