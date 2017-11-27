#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path

rospy.init_node("nav_grabber")

plan_publisher = rospy.Publisher('new_plan', Path, queue_size=10)

rate = rospy.Rate(10.0)

def get_plan():

	new_plan = Path()
	new_plan.header.frame_id = 'map'

	plan = rospy.wait_for_message('move_base/NavfnROS/plan', Path)
	print "Publishing a new plan..."
	for i in range(len(plan.poses)):
		if i % 200 == 0:
			new_plan.poses.append(plan.poses[i])

	plan_publisher.publish(new_plan)

while not rospy.is_shutdown():
	get_plan()
	rate.sleep()
