#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from math import pow,atan2,sqrt
from nav_msgs.msg import Odometry
import tf
from math import radians, degrees, sqrt
from time import sleep
import move_base_msgs.msg
import actionlib
from ca_msgs.msg import Bumper
from nav_msgs.msg import Path
from std_msgs.msg import Empty

home_x = -4
home_y = 7.5
final_orientation = -1*radians(90)

rospy.init_node("create_2_return_to_dock")

velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
dock_publisher = rospy.Publisher('dock',Empty, queue_size=10)
undock_publisher = rospy.Publisher('undock',Empty, queue_size=10)

rate = rospy.Rate(10.0)

listener = tf.TransformListener()

def move_base_client():

	client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	client.wait_for_server()
	goal = move_base_msgs.msg.MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = home_x
	goal.target_pose.pose.position.y = home_y
	goal.target_pose.pose.orientation.z = 0.0
	goal.target_pose.pose.orientation.w = 1.0
	
	print "Sending goal..."
	client.send_goal(goal)
	#client.wait_for_result()
	#result = client.get_result()

def get_plan():

	new_plan = []

	move_base_client()

	plan = rospy.wait_for_message('move_base/NavfnROS/plan', Path)
	print "Calculating a new plan..."
	for i in range(1,len(plan.poses)):
		if i % 200 == 0:
			point = plan.poses[i]
			x = point.pose.position.x
			y = point.pose.position.y
			new_point = [x,y]
			new_plan.append(new_point)
	new_plan.append([home_x,home_y])
	return new_plan

def check_camera():
	
	got_one = False

	while not got_one:

		try:
			(t,rot) = listener.lookupTransform('/map', '/roomba3', rospy.Time(0))
			got_one = True

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	euler = tf.transformations.euler_from_quaternion(rot)
	yaw = euler[2]
	quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
	trans = (t[0],t[1],0.0)
	return (trans,quat)

def publish_cmd_vel(lin_vel, angle_vel):
	vel_msg = Twist()
	vel_msg.linear.x = lin_vel
	vel_msg.angular.z = angle_vel
	velocity_publisher.publish(vel_msg)

def bumperData():
	bumper_msg = rospy.wait_for_message("bumper", Bumper)
	check = False

	if (bumper_msg.is_left_pressed or bumper_msg.is_right_pressed):
		check = True

	return check 

def moveTo(distance):
	publish_cmd_vel(0.0,0.0)
	sleep(1)
	tries = 0
	kp = 0.45
	ki = 0.005
	kd = 0
	integral = 0
	last_error = 0
	derivative = 0

	i = 0

	target_achieved = False

	target = distance 
	(trans,rot) = check_camera()
	x_init = trans[0]
	y_init = trans[1]

	while not target_achieved:
		
		check = bumperData()

		if check == True:
			break

		(trans,quat) = check_camera()
		cur_x = trans[0]
		cur_y = trans[1]
		cur_d = sqrt((cur_x - x_init)**2 + (cur_y - y_init)**2)
		error = abs(target) - cur_d
		print "Error: "+ str(error)
		integral += error
		derivative = error - last_error
		if (error>0 and last_error <0) or (error<0 and last_error>0):
			integral = 0
			publish_cmd_vel(0.0,0.0)
			print "***Flipped Integral***"
			sleep(1)
		if derivative >= 0.05:
			integral = 0
		print "Integral: " + str(integral)
		print "Derivative: " + str(derivative)
		lin_vel = kp*error + ki*integral + kd*derivative
		print "Lin_vel: " + str(lin_vel)
		if abs(error) < 0.02:
			lin_vel = 0
			i += 1
		publish_cmd_vel(lin_vel,0)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()
		tries += 1
		if tries >= 300:
			target_achieved = True
			print "Giving up"
			publish_cmd_vel(0,0)

	check = bumperData()

	if check == True:
		for i in range(0,5):
			publish_cmd_vel(0,0)
			rate.sleep()
		sleep(2)	
		for i in range(0,10):
			publish_cmd_vel(-0.2, 0)
			rate.sleep()

		publish_cmd_vel(0,0)

def rotateTo(angle):
	publish_cmd_vel(0.0,0.0)
	sleep(1)
	tries = 0
	kp = 0.5
	ki = 0.06
	kd = 0.0
	
	integral = 0
	last_error = 0
	derivative = 0
	i = 0

	target_achieved = False

	target = angle
	while not target_achieved:

		check = bumperData()
		if check == True:
			break
		(trans,quat) = check_camera()
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = euler[2]
		error = target - yaw
		if error > radians(180):
			error = error - radians(360)
		if error < radians(-180):
			error = error + radians(360)
		print "Error: "+ str(error)
		integral += error
		if (error > 0 and last_error < 0) or (error<0 and last_error>0):
			intergal = 0.0
			print "Flipped INTEGRAL***"
		derivative = error - last_error
		if abs(derivative)*1000 > 10:
			integral = 0.0
		ang_vel = kp*error + ki*integral + kd*derivative
		print "Integral: " + str(integral)
		print "Ang_vel: " + str(ang_vel)
		print "Target angle: " + str(degrees(target)) 
		if abs(error) < 0.02: #old threshold =0.006
			integral = 0
			ang_vel = 0
			i += 1
		publish_cmd_vel(0,ang_vel)
		last_error = error
		print "---"
		if i >= 5:
			target_achieved = True
		rate.sleep()
		tries += 1
		if tries >= 300:
			target_achieved = True
			print "Giving up"
			publish_cmd_vel(0,0)

def moveBreak(target_x,target_y):

	(trans, rot) = check_camera()
	x_init = trans[0]
	y_init = trans[1]
	
	points = []

	total_x = target_x-x_init
	total_y = target_y-y_init
	distance = sqrt((total_x)**2+(total_y)**2)
	step = 1
	d = step

	while d < distance:

		step_x = (d*total_x)/distance
		step_y = (d*total_y)/distance
		x = x_init + step_x
		y = y_init + step_y
		points.append([x,y])
		d += step

	points.append([target_x,target_y])
	print points
	return points

def move2goal(points):

	for point in points: 

		point_x = point[0]
		point_y = point[1]

		(trans, rot) = check_camera()
		current_x = trans[0]
		current_y = trans[1]
		direction = atan2((point_y-current_y),(point_x-current_x))
		print degrees(direction)


		rotateTo(direction)
		sleep(2)
		
		(trans, rot) = check_camera()
		current_x = trans[0]
		current_y = trans[1]
		movement_distance = sqrt((point_x-current_x)**2+(point_y-current_y)**2)
		moveTo(movement_distance)

return_to_dock = False
while not return_to_dock:
	return_to_dock = rospy.get_param('return_to_dock', False)
	sleep(1)
x = Empty()
print check_camera()
move2goal(get_plan())
sleep(2)
rotateTo(final_orientation)
sleep(2)
dock_publisher.publish(x)

#while not rospy.is_shutdown():
#	print check_camera()
#	move2goal(get_plan)