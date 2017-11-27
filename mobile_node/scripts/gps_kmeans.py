#!/usr/bin/env python

import numpy as np
import math
import Tkinter as tk
import rospy
from geometry_msgs.msg import Vector3
from itertools import combinations

# h is height to ground from tags in meters
#h = 1.9558
h = 2.5
#h = 1.95
# n is gride size
n = 4
# step size is distance between nodes in the grid (here 3 feet = 0.9144 meters)
step_size = 0.9144
# Display offset zoom factor
zoom = 100
# Display centering fudge factor
offset = 100

ran = 0
var = 100
count = 0

root = tk.Tk()
canvas = tk.Canvas(root, width=500, height=500, borderwidth=0, highlightthickness=0, bg="white")
canvas.grid()
root.wm_title("Grid Positioning System")


def gps_callback(gps_data):
	#rospy.loginfo("I heard tag_%s at a distance of " + str(gps_data.y), gps_data.x)
	global ran
	global var
	ran = int(gps_data.x)
	var = (gps_data.y/1000)

def listener():
	rospy.init_node('gps_listener', anonymous=True)
	rospy.Subscriber("gps_data", Vector3, gps_callback)

def _create_circle(self, x, y, r, **kwargs):
	return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle

def _create_circle_arc(self, x, y, r, **kwargs):
	if "start" in kwargs and "end" in kwargs:
		kwargs["extent"] = kwargs["end"] - kwargs["start"]
		del kwargs["end"]
		return self.create_arc(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle_arc = _create_circle_arc

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

pts = list()
circles = list()

for k in range(n*n):
	col = k % n
	row = (k - col) / n
	x = col*step_size
	y = row*step_size
	pts.append([x,y])
	circles.append(k)
	label = tk.Label(canvas, text=("tag_" + str(k)))
	label.place(x=(x*zoom + offset), y=(y*zoom + offset))
	canvas.create_circle((x*zoom + offset), (y*zoom + offset), 5, fill="black", outline="#DDD", width=4)

	#print pts[k]
	#print "col: " + str(col) + " row: " + str(row)

tag = list()
#tag.append([0,0,2.393])
#print var
#print h
# pos_update will update the grid of guesses about your current position
def pos_update(tag,d):
	global count
	points = list()
	if tag >=0 and tag <= ((n*n)-1):
		x = pts[tag][0]
		y = pts[tag][1]
		r = math.sqrt((d*d) - (h*h))
		circles[tag]=[x,y,r]
		name = "tag_" + str(tag)
		canvas.delete(name)

		if count >= 600:

			tuples = list()
			#print len(circles)
			for i in range(0, len(circles)):
				if type(circles[i]) != int and len(circles[i]) == 3:
					tuples.append(i)	

			it = combinations(tuples, 2)
			#print triplets
			k = 0
			for i in it:
				## find intersection pts of circles	
				#points.insert(k,find_intersect(circles[i[0]],circles[i[1]])) 
				print i
				print circles[i[0]]
				print circles[i[1]]
				k += 1

			print points	
			
			for i in range((n*n)):
				canvas.delete(("tag_" + str(i)))
				count = 0
		
		canvas.create_circle((x*zoom + offset), (y*zoom + offset), (r*zoom), outline="blue", width=1, tag=name)
		canvas.after(0)
		canvas.update()
		count += 1
		#print ran
		#print var
		
		#print circles
	else:
		print "You have entered an invalid tag"

# distance function finds the distance between two points
def distance(p1,p2):
	x_sqrd = (p1[0]-p2[0])**2
	y_sqrd = (p1[1]-p2[1])**2
	dist = math.sqrt(x_sqrd + y_sqrd)
	return dist


def Main():
	while True:
		listener()
		pos_update(ran,var)


if __name__ == '__main__':
	Main()