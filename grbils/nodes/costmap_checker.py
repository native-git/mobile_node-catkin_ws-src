#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from bresenham import bresenham

rospy.init_node("costmap_checker")

costmap = rospy.wait_for_message("move_base/global_costmap/costmap", OccupancyGrid)

frame_id = costmap.header.frame_id
resolution = costmap.info.resolution # resolution in meters/cell
width = costmap.info.width # number of cells
height = costmap.info.height # number of cells

x_offset = costmap.info.origin.position.x
y_offset = costmap.info.origin.position.y

def get_pixel_number((x_cell,y_cell)):
	pixel_number = y_cell * width + x_cell
	return pixel_number

def to_cell_pos(x,y):
	x -= x_offset
	y -= y_offset
	x_cell = int(round(x/resolution))
	y_cell = int(round(y/resolution))
	return (x_cell,y_cell)
	#return costmap.data[pixel_number]
	#print x_cell,y_cell

def check_line_between(x0,y0,xf,yf):
	(x_init,y_init) = to_cell_pos(x0,y0)
	(x_final,y_final) = to_cell_pos(xf,yf)
	points = list(bresenham(x_init,y_init,x_final,y_final))
	for point in points:
		print costmap.data[get_pixel_number(point)]
	#print points

check_line_between(-7.4,8.1,-5.7,10.6)


#print costmap.info
#print len(costmap.data)