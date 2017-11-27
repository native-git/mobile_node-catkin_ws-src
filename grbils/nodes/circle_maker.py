#!/usr/bin/env python

from math import radians,cos,sin

x0 = -3.0
y0 = 8.0
r = 2.5

def calc_point(theta):
	x = x0 + r*cos(theta)
	y = y0 + r*sin(theta)
	return (x,y)

for i in range(0,15):
	theta = radians(180) - radians(90/8)*i
	(x,y) = calc_point(theta)
	print str(x) + ","+str(y)+",True"