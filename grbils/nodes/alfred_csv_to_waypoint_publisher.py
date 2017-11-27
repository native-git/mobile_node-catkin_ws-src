#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
import csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from bresenham import bresenham
import tf

filename = './planned_path.csv'

rospy.init_node("alfred_csv_to_waypoints")

plan_publisher = rospy.Publisher('alfred/waypoints/queue', Path, queue_size=10)

make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()

start_x = -8
start_y = 8.6

costmap = rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid)

frame_id = costmap.header.frame_id
resolution = costmap.info.resolution # resolution in meters/cell
width = costmap.info.width # number of cells
height = costmap.info.height # number of cells

x_offset = costmap.info.origin.position.x
y_offset = costmap.info.origin.position.y

def check_camera():
	
	got_one = False

	while not got_one:

		try:
			(t,rot) = listener.lookupTransform('/map', '/alfred', rospy.Time(0))
			got_one = True

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[2]
	quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
	trans = (t[0],t[1],0.0)
	return (trans,quat)

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
	valid = True
	(x_init,y_init) = to_cell_pos(x0,y0)
	(x_final,y_final) = to_cell_pos(xf,yf)
	points = list(bresenham(x_init,y_init,x_final,y_final))
	for point in points:
		value = costmap.data[get_pixel_number(point)]
		if value > 5:
			valid = False
	return valid


def eval_string(statement):
	if statement == "True":
		return True
	else:
		return False

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

	return plan.plan.poses

def level_0_check(points):
	x_min = -15
	x_max = 1
	y_min = 5
	y_max = 19
	valid = True
	for i in range(len(points)):
		x = float(points[i][0])
		y = float(points[i][1])
		if x < x_min or x > x_max or y < y_min or y > y_max:
			print "There following point is invalid from file: '" + str(filename) + "' on line " + str(i)
			print ">> "+str(points[i][0])+","+str(points[i][1])+","+str(points[i][2])
			valid = False
		if x < x_min or x > x_max:
			print "--REASON: x value: " + str(x)+" falls outside of usable range: "+str(x_min)+" <= x <= " + str(x_max)
			if y>y_min and y<y_max:
				print ""
		if y < y_min or y > y_max:
			print "--REASON: y value: " + str(y)+" falls outside of usable range: "+str(y_min)+" <= x <= " + str(y_max)+"\n"
	return valid

def level_1_check(points):
	valid = True
	for i in range(len(points)):
		x = float(points[i][0])
		y = float(points[i][1])
		px_num = get_pixel_number(to_cell_pos(x,y))
		if costmap.data[px_num] > 5:
			print "There following point is invalid from file: '" + str(filename) + "' on line " + str(i)
			print ">> "+str(points[i][0])+","+str(points[i][1])+","+str(points[i][2])
			print "--REASON: Point falls on a known obstacle\n"
			valid = False
	return valid

def level_2_check(points):
	valid = True
	for i in range(1,len(points)):
		x = float(points[i][0])
		y = float(points[i][1])
		path_important = eval_string(points[i][2])
		if path_important:
			x_prev = float(points[i-1][0])
			y_prev = float(points[i-1][1])
			path_ok = check_line_between(x_prev,y_prev,x,y)
			if not path_ok:
				print "The line segment from file: '" + str(filename) + "' is invalid on lines " + str(i-1) + " & " + str(i)
				print ">> "+str(points[i-1][0])+","+str(points[i-1][1])+","+str(points[i-1][2])
				print ">> "+str(points[i][0])+","+str(points[i][1])+","+str(points[i][2])
				print "--REASON: The path given intersects a known obstacle\n"
				valid = False
	return valid

def publish_plan():

	new_plan = Path()
	new_plan.header.frame_id = 'map'

	with open(filename) as csvfile:
		points_reader = csv.reader(csvfile, delimiter = ',')
		points_list = list(points_reader)

		test0 = level_0_check(points_list)
		test1 = level_1_check(points_list)
		test2 = level_2_check(points_list)

		print test0,test1,test2

		if test0 and test1 and test2:
			(trans,rot) = check_camera()
			#response = get_plan(start_x,start_y,float(points_list[0][0]),float(points_list[0][1]))
			response = get_plan(trans[0],trans[1],float(points_list[0][0]),float(points_list[0][1]))
			for i in range(len(response)):
				if i == 50:
					new_plan.poses.append(response[i])
				if i % 200 == 0 and i != 0:
					new_plan.poses.append(response[i])
			new_plan.poses.append(response[-1])

			for i in range(1,len(points_list)):
				x = float(points_list[i][0])
				y = float(points_list[i][1])
				path_important = eval_string(points_list[i][2])
				#path_important = bool(points_list[i][2])

				if not path_important:
					prev_x = float(points_list[i-1][0])
					prev_y = float(points_list[i-1][1])
					response = get_plan(prev_x,prev_y,x,y)
					for i in range(len(response)):
						if i % 200 == 0:
							new_plan.poses.append(response[i])
					new_plan.poses.append(response[-1])
				else:
					point = PoseStamped()
					point.pose.position.x = x
					point.pose.position.y = y
					new_plan.poses.append(point)

			print "Running final verification of plan..."
			path_is_clear = True
			for i in range(len(new_plan.poses)-1):
				xi = new_plan.poses[i].pose.position.x
				yi = new_plan.poses[i].pose.position.y
				xf = new_plan.poses[i+1].pose.position.x
				yf = new_plan.poses[i+1].pose.position.y
				valid = check_line_between(xi,yi,xf,yf)
				if not valid:
					path_is_clear = False
					print "Path segment invalid"
					print "first_point: "
					print xi,yi
					print "second_point: "
					print xf,yf
			if path_is_clear:
				print "Publishing the final plan..."
				plan_publisher.publish(new_plan)
			else:
				print "Plan needs to be reworked, something went wrong"
	#print new_plan

publish_plan()