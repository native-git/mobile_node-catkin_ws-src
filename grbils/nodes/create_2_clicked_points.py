#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import csv
from geometry_msgs.msg import PoseStamped, PointStamped
import threading

rospy.init_node("create_2_clicked_points")

plan_publisher = rospy.Publisher('/roomba3/waypoints', Path, queue_size=10)
temp_plan_publisher = rospy.Publisher('/roomba3/temporary_path', Path, queue_size=10)

rate = rospy.Rate(10.0)

points = []

def publish_plan():
	global points
	new_plan = Path()
	new_plan.header.frame_id = 'map'
	for point in points:
		way_point = PoseStamped()
		way_point.pose.position.x = point[0]
		way_point.pose.position.y = point[1]
		new_plan.poses.append(way_point)
	plan_publisher.publish(new_plan)

def publish_temp_plan():
	global points
	new_plan = Path()
	new_plan.header.frame_id = 'map'
	for point in points:
		way_point = PoseStamped()
		way_point.pose.position.x = point[0]
		way_point.pose.position.y = point[1]
		new_plan.poses.append(way_point)
	temp_plan_publisher.publish(new_plan)

def wait_for_key():

	global points

	keep_running = True

	while keep_running:
		x = raw_input("Press enter when you want to save your path, <c> to clear everything, or <d> to delete last point...:  ")
		if x == 'd':
			if len(points)>= 1:
				points = points[:-1]
				publish_temp_plan()
		if x == 'c':
			del points[:]
		if x == "":
			print "publishing the plan"
			publish_plan()
			keep_running = False

thread = threading.Thread(target=wait_for_key)
thread.daemon = True
thread.start()

while thread.is_alive():
	while not rospy.is_shutdown():
		global points
		point = rospy.wait_for_message('roomba3/clicked_point', PointStamped)
		x = point.point.x 
		y = point.point.y
		points.append([x,y])
		publish_temp_plan()
		