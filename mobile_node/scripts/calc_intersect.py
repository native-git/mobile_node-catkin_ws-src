#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import numpy as np
import math
import Tkinter as tk

circles = list()

height = 2.5
# n is gride size
n = 4
# step size is distance between nodes in the grid (here 3 feet = 0.9144 meters)
step_size = 0.9144

def distance(p0,p1):
	x_sqrd = (p0[0]-p1[0])**2
	y_sqrd = (p0[1]-p1[1])**2
	dist = math.sqrt(x_sqrd + y_sqrd)
	return dist

def find_intersect(c0,c1):
	
	#pX[0] is x coordinate of circle pX likewise for y

	p0 = [c0[0],c0[1]]
	p1 = [c1[0],c1[1]]
	r0 = c0[2]
	r1 = c1[2]
	d = distance(p0,p1)

	if d > r0 + r1 or d < abs(r0 - r1):
		print "Error --> the two circles do not intersect for some odd reason"
	elif d == 0 and r0 == r1:
		print "The circles are coincident"
	else:
		intersects = list()
		a = ((r0**2 - r1**2 + d**2)/(2*d))
		h = math.sqrt(r0**2 - a**2)

		x_val = h*(p1[1]-p0[1])/d
		y_val = h*(p1[0]-p0[0])/d

		x2 = (-1 * a/d)*(p0[0]-p1[0]) + p0[0]
		y2 = (-1 * a/d)*(p0[1]-p1[1]) + p0[1]

		sol_1 = [(x2 + x_val),(y2 - y_val)]
		sol_2 = [(x2 - x_val),(y2 + y_val)]

		intersects.append(sol_1)
		intersects.append(sol_2)
		return intersects

def update_circles(tag,radius):
	circle = circles[tag]
	r = math.sqrt((radius**2) - (height**2))
	circles[tag].insert(2, r)

def Main():

	for k in range(n*n):
		col = k % n
		row = (k - col) / n
		x = col*step_size
		y = row*step_size
		circles.append([x,y])

	
	update_circles(6,(3004.89343192/1000))
	update_circles(8,(2752.34869479/1000))
	print circles

	points = find_intersect(circles[6],circles[8])
	print "Points----"
	print points


if __name__ == '__main__':
	Main()
"""
I heard tag_6.0 at a distance of 3004.89343192

I heard tag_8.0 at a distance of 2752.34869479
"""
