#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

rospy.init_node("generate_path_plan")

path_publisher = rospy.Publisher('temporary_plan', Path, queue_size=10)

rate = rospy.Rate(10.0)

make_plan = rospy.ServiceProxy("move_base/make_plan", GetPlan)


def get_plan(x0,y0,xf,yf):

	start = PoseStamped()
	goal = PoseStamped()
	tolerance = 0.0

	start.header.frame_id = "map"
	goal.header.frame_id = "map"

	start.pose.position.x = x0
	start.pose.position.y = y0

	goal.pose.position.x = xf
	goal.pose.position.y = yf

	plan = make_plan(start,goal,tolerance)

	plan.plan.header.frame_id = "map"
	print plan.plan.poses
	return plan.plan

get_plan(-10.0,12.0,-3.0,10.0)

"""
start.pose.position.x = -10.0
start.pose.position.y = 12.0

goal.pose.position.x = -3.0
goal.pose.position.y = 10.0

plan = make_plan(start,goal,tolerance)

plan.plan.header.frame_id = "map"
path_publisher.publish(plan.plan)
"""