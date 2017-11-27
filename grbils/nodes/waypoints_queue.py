#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist, PoseStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry, Path
from time import sleep
import threading

rospy.init_node("waypoints_queue")
path_publisher = rospy.Publisher('/roomba3/waypoint', Path, queue_size=10)

rate = rospy.Rate(10.0)

def cancel_plan():
	test = rospy.wait_for_message('cancel_plan', Empty)
	
def publish_waypoint(waypoint):
	path = Path()
	path.header.frame_id = 'map'
	way_point = PoseStamped()
	way_point.pose.position.x = waypoint[0]
	way_point.pose.position.y = waypoint[1]
	path.poses.append(way_point)
	path_publisher.publish(path)

def get_plan():

	new_plan = []

	plan = rospy.wait_for_message('roomba3/waypoints/queue', Path)
	print "Calculating a new plan..."
	for point in plan.poses:
		x = point.pose.position.x
		y = point.pose.position.y
		new_point = [x,y]
		new_plan.append(new_point)
	return new_plan

while not rospy.is_shutdown():
	new_plan = get_plan()
	thread = threading.Thread(target=cancel_plan)
	thread.daemon = True
	if not thread.is_alive():
		thread.start()

	for waypoint in new_plan:
		if not thread.is_alive():
			break
		print waypoint
		rospy.set_param('target_achieved',False)
		rospy.set_param('target_received', False)
		target_received = False
		sleep(1)
		publish_waypoint(waypoint)
		target_achieved = rospy.get_param('target_achieved')
		while not target_achieved:
			sleep(0.25)
			target_achieved = rospy.get_param('target_achieved')
			if not thread.is_alive():
				target_achieved = False
				break
