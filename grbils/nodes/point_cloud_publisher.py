#!/usr/bin/env python
import rospy
import math
import sys
import random

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

rospy.init_node("point_cloud_publisher")

obstacle_pub = rospy.Publisher("obstacle1", PointCloud, queue_size=10)

rate = rospy.Rate(10.0)

def publish_cloud():
	pcl = PointCloud()
	pcl.header.frame_id = "map"
	pcl.header.stamp = rospy.Time.now()

	pcl.points = [None] * 10
	for i in range(0,len(pcl.points)):
		x = random.uniform(-2,2)
		y = random.uniform(-2,2)
		z = 0.01
		pcl.points[i] = Point32(x,y,z)

	obstacle_pub.publish(pcl)

while not rospy.is_shutdown():
	publish_cloud()
	rate.sleep()