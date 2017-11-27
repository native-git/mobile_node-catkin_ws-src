#! /usr/bin/env python

import csv
from sys import argv

x_range = [0,15] # range of acceptable x coordinates in [meters]
y_range = [0,15] # range of acceptable y coordinates in [meters]
yaw_range = [-180,180] # orientation range in [degrees]

proceed	= True
if len(argv) != 2:
	proceed = False
	print """
Usage: ./waypoints.py <filename>
		
where <filename> is the name of a .csv file containing waypoints with the following format:
	x,y,yaw,sleep_time
	x2,y2,yaw2,sleep_time2
	etc...

x_range = [0,5] # range of acceptable x coordinates in [meters]
y_range = [0,5] # range of acceptable y coordinates in [meters]
yaw_range = [-180,180] # orientation range in [degrees]
		"""

elif argv[1] == '-h' or argv[1] == '--help':
	proceed = False
	print """
Usage: ./waypoints.py <filename>
	
where <filename> is the name of a .csv file containing waypoints with the following format:
	x,y,yaw,sleep_time
	x2,y2,yaw2,sleep_time2
	etc...
	"""

if len(argv) == 2:
	filename = str(argv[1])

def check_row(row):
	
	if len(row) != 4:
		return (False, "Wrong number of arguments, expected 4, got " + str(len(row)))
	x = float(row[0])
	y = float(row[1])
	yaw = float(row[2])
	sleep = float(row[3])
	if x < x_range[0] or x > x_range[1]:
		return (False, "x value outside of acceptable range, " + str(x_range))
	if y < y_range[0] or y > y_range[1]:
		return (False, "y value outside of acceptable range, " + str(y_range))
	if yaw < yaw_range[0] or yaw > yaw_range[1]:
		return (False, "y value outside of acceptable range, " + str(yaw_range))
	else:
		return (True,"")
if proceed:
	with open(filename,'rb') as f:
		valid = True
		waypoints = csv.reader(f)
		for row in waypoints:
			status = check_row(row)
			if status[0] == False:
				valid = False
				print ""
				print  "Error at line: ",waypoints.line_num," of, ",filename,", Type: ",status[1]
				print "\t",row,"\n"
		if valid:
			print "You have input a valid list of waypoints..."