#!/usr/bin/env python

import serial
import time
import struct
import threading
from math import pi, degrees, radians, sin, cos
from numpy import sign
import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String, Int16, Float32, Bool
from roscpp_tutorials.srv import TwoInts, TwoIntsResponse

br = tf.TransformBroadcaster()

port = '/dev/ttyUSB0'

set_v = 0
l_const = 1
r_const = 1
linear = False

mm_per_tick = (72.0 * pi)/508.8

clear_for_service = False
rotate_request = False
move_request = False

lock = threading.Lock()
service_lock = threading.Lock()

rospy.init_node("create_pid_ros_node")
# Setup the publishers
charging_state_publisher = rospy.Publisher("charging_state", String, queue_size=10)
mode_publisher = rospy.Publisher("current_mode", String, queue_size=10)
battery_voltage_publisher = rospy.Publisher("battery/voltage", Float32, queue_size=10)
battery_charge_ratio_publisher = rospy.Publisher('battery/charge_ratio', Float32, queue_size=10)
bumper_publisher = rospy.Publisher('bumper', Bool, queue_size=10)
wheel_drop_publisher = rospy.Publisher('wheel_drop', Bool, queue_size=10)

get_bin = lambda x, n: format(x, 'b').zfill(n)

#sensors = {'wheel_drop_left':None,'wheel_drop_right':None,'left_bumper':None,'right_bumper':None,'left_ticks':None,'right_ticks':None}
sensors = {}
group = {}
group.update({'bumpers':['wheel_drop_left','wheel_drop_right','left_bumper','right_bumper']})
group.update({'encoders':['left_ticks','right_ticks']})
group.update({'charging_state':['not_charging','reconditioning_charging','full_charging','trickle_charging','waiting','charging_fault_condition']})
group.update({'battery':['battery_voltage','battery_charge','battery_capacity']})
group.update({'mode':['Off','Passive','Safe','Full']})
mode_op_codes = {}
mode_commands = ['173','128','131','132']
mode_op_codes.update(dict(zip(group['mode'],mode_commands)))
connection = serial.Serial(port, baudrate=115200, timeout=1)

high_rate = rospy.Rate(5.0)
low_rate = rospy.Rate(1.0)

def broadcast_tf(x,y,theta):
	br.sendTransform((x,y,0),tf.transformations.quaternion_from_euler(0,0,theta),rospy.Time.now(),'roomba3/base_link','roomba3/odom')

def publish_charging_state():
	msg = String()
	for i in group['charging_state']:
		if sensors[i] == True:
			msg.data = i
			charging_state_publisher.publish(msg)

def publish_mode():
	msg = String()
	for i in group['mode']:
		if sensors[i] == True:
			msg.data = i
			mode_publisher.publish(msg)

def publish_battery_voltage():
	msg = Float32()
	msg.data = float(sensors['battery_voltage'])/1000.0
	battery_voltage_publisher.publish(msg)

def publish_battery_charge_ratio():
	msg = Float32()
	ratio = float(sensors['battery_charge'])/float(sensors['battery_capacity'])
	msg.data = ratio
	battery_charge_ratio_publisher.publish(msg)

def publish_bumpers():
	msg = Bool()
	if sensors['left_bumper'] == True or sensors['right_bumper'] == True:
		msg.data = True
	else:
		msg.data = False
	bumper_publisher.publish(msg)

def publish_wheel_drop():
	msg = Bool()
	if sensors['wheel_drop_left'] == True or sensors['wheel_drop_right'] == True:
		msg.data = True
	else:
		msg.data = False
	wheel_drop_publisher.publish(msg)

def cmd_vel_callback(data):
	global set_v, l_const, r_const, linear
	lin_vel = data.linear.x
	ang_vel = data.angular.z
	if lin_vel == 0.0 and ang_vel != 0.0:
		linear = False
		lin_v = ang_vel * 117.5 # 117.5 is 0.5*distance between both wheels
		if lin_v > 500:
			lin_v = 500
		if lin_v < -500:
			lin_v = -500
		set_v = int(lin_v)
		l_const = -1
		r_const = 1
	if ang_vel == 0.0 and lin_vel != 0.0:
		linear = True
		lin_vel *= 1000
		if lin_vel > 500:
			lin_vel = 500
		if lin_vel < -500:
			lin_vel = -500
		set_v = int(lin_vel)
		l_const = 1
		r_const = 1
	if ang_vel == 0.0 and lin_vel == 0.0:
		linear = False
		set_v = 0
		l_const = 1
		r_const = 1

def mode_callback(msg):
	if msg.data >= 0 and msg.data <= 3:
		with lock:
			send_command(mode_commands[int(msg.data)])

def dock_callback(data):
	with lock:
		send_command('143')

def rotate_callback(req):
	global rotate_request, clear_for_service
	rotate_request = True
	theta = radians(req.a)
	d_ticks = (117.5*theta)/mm_per_tick
	d_ticks = int(d_ticks)
	max_dPWM = 1

	while not clear_for_service:
		time.sleep(0.05)
	if clear_for_service:
		time.sleep(0.25)
		(start_left,start_right) = get_sensor_values()

		d_l_targ = -d_ticks
		d_r_targ = d_ticks

		d_ticks_l = 0
		d_ticks_r = 0

		print d_l_targ,d_r_targ

		avg_error = d_ticks

		PWM_raw = 0

		r_pwm = 0
		l_pwm = 0

		tolerance = 5

		i_max = 250
		i = 0
		target_achieved = False

		with service_lock:

			while not target_achieved:

				i += 1

				(left,right) = get_sensor_values()

				dl = left - start_left
				dr = right - start_right

				if dl<-32768:
					dl = -dl - 32768
				if dr<-32768:
					dr = -dr - 32768
				if dl > 32767:
					dl -= 32767
				if dr > 32767:
					dr -= 32767

				d_ticks_l = dl
				d_ticks_r = dr

				l_error = d_l_targ - d_ticks_l
				r_error = d_r_targ - d_ticks_r

				avg_error = (abs(r_error)+abs(l_error))*0.5

				#if abs(l_error)<tolerance and abs(r_error)<tolerance:
				if abs(avg_error)<=tolerance:
					drive_direct_command(0,0)
					target_achieved = True
					print avg_error
					print "D_ticks_l: "+str(d_ticks_l)+" D_ticks_r: "+str(d_ticks_r)
					print "target_achieved"

				if i>=i_max:
					drive_direct_command(0,0)
					target_achieved = True
					print "Failed to reach target within time limit"
					avg_error = -1

				else:
					diff = d_ticks_l + d_ticks_r
					dpwm_rel = min(abs(diff)*2,20)*-1*sign(diff)

					if d_ticks_l or d_ticks_r <=5:
						set_point = min(int(round(avg_error*4.25)),255)
					else:
						set_point = min(int(round(avg_error*2.5)),255)

					if set_point > (PWM_raw+max_dPWM):
						PWM_raw += max_dPWM
					if PWM_raw > set_point:
						PWM_raw = set_point

					l_pwm = (PWM_raw*sign(l_error))+dpwm_rel
					r_pwm = (PWM_raw*sign(r_error))+dpwm_rel

					if abs(l_pwm) > 255:
						l_pwm = 255*sign(l_pwm)
					if abs(r_pwm) > 255:
						r_pwm = 255*sign(r_pwm)

					print d_ticks
					print "PWM_raw: "+str(PWM_raw)
					print "D_ticks_l: "+str(d_ticks_l)+" D_ticks_r: "+str(d_ticks_r)
					print "dpwm_rel: " + str(dpwm_rel)
					print "l_pwm: "+str(l_pwm)+" r_pwm: "+str(r_pwm)
					print "avg_error: "+str(avg_error)
					print "--"
					drive_pwm(l_pwm,r_pwm)
				time.sleep(0.05)
			rotate_request = False
	return TwoIntsResponse(avg_error)


def move_callback(req):
	global move_request, clear_for_service
	move_request = True
	dist = req.a
	d_ticks = int(round((dist/mm_per_tick)))
	max_dPWM = 1

	while not clear_for_service:
		time.sleep(0.05)
	if clear_for_service:
		time.sleep(0.25)
		
		(start_left,start_right) = get_sensor_values()
		
		d_ticks_l = 0
		d_ticks_r = 0

		avg_error = d_ticks

		PWM_raw = 0

		r_pwm = 0
		l_pwm = 0

		i_max = 250
		i = 0

		tolerance = 5

		target_achieved = False

		with service_lock:

			while not target_achieved:

				i += 1

				(left,right) = get_sensor_values()

				dl = left - start_left
				dr = right - start_right

				if dl<-32768:
					dl = -dl - 32768
				if dr<-32768:
					dr = -dr - 32768
				if dl > 32767:
					dl -= 32767
				if dr > 32767:
					dr -= 32767

				d_ticks_l = dl
				d_ticks_r = dr

				l_error = d_ticks - d_ticks_l
				r_error = d_ticks - d_ticks_r

				average_ticks = (dl+dr)*0.5

				d_l_avg = int(round(average_ticks - dl))
				d_r_avg = int(round(average_ticks - dr))

				dpwm_rel_l = min(abs(d_l_avg),10)*sign(d_l_avg)
				dpwm_rel_r = min(abs(d_r_avg),10)*sign(d_r_avg)

				avg_error = (abs(r_error)+abs(l_error))*0.5

				if abs(avg_error)<=tolerance:
					drive_direct_command(0,0)
					target_achieved = True
					print avg_error
					print "D_ticks_l: "+str(d_ticks_l)+" D_ticks_r: "+str(d_ticks_r)
					print "target_achieved"

				if i>=i_max:
					drive_direct_command(0,0)
					target_achieved = True
					print "Failed to reach target within time limit"
					avg_error = -1

				else:
					if d_ticks_l or d_ticks_r <=5:
						set_point = min(int(round(avg_error*4)),255)
					else:
						set_point = min(int(round(avg_error*2.5)),255)
					if set_point > (PWM_raw+max_dPWM):
						PWM_raw += max_dPWM
					if PWM_raw > set_point:
						PWM_raw = set_point

					l_pwm = (PWM_raw*sign(l_error))+dpwm_rel_l
					r_pwm = (PWM_raw*sign(r_error))+dpwm_rel_r

					if abs(l_pwm) > 255:
						l_pwm = 255*sign(l_pwm)
					if abs(r_pwm) > 255:
						r_pwm = 255*sign(r_pwm)

					print d_ticks
					print "PWM_raw: "+str(PWM_raw)
					print "D_ticks_l: "+str(d_ticks_l)+" D_ticks_r: "+str(d_ticks_r)
					print "dpwm_rel_l: " + str(dpwm_rel_l) + "dpwm_rel_r: " + str(dpwm_rel_r)
					print "l_pwm: "+str(l_pwm)+" r_pwm: "+str(r_pwm)
					print "avg_error: "+str(avg_error)
					print "--"
					drive_pwm(l_pwm,r_pwm)
				time.sleep(0.05)
			move_request = False
	return TwoIntsResponse(avg_error)


def subscribers():
	rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
	rospy.Subscriber("mode", Int16, mode_callback)
	rospy.Subscriber("dock", Empty, dock_callback)
	rospy.Service('rotate', TwoInts, rotate_callback)
	rospy.Service('move', TwoInts, move_callback)
	rospy.spin()

def low_frequency_publishers():
	while not rospy.is_shutdown():
		publish_charging_state()
		publish_mode()
		publish_battery_voltage()
		publish_battery_charge_ratio()
		low_rate.sleep()

def high_frequency_publishers():
	while not rospy.is_shutdown():
		publish_bumpers()
		publish_wheel_drop()
		high_rate.sleep()

def send_command(command):
	global connection
	connection.reset_output_buffer()
	cmd = ""
	for byte in command.split():
		cmd += chr(int(byte))
	try:
		if connection is not None:
			connection.write(cmd)
		else:
			print "Not connected."
	except serial.SerialException:
		print "Lost connection"

def drive_direct_command(v_right,v_left):
	global connection
	cmd = struct.pack(">Bhh", 145, v_right, v_left)
	command = ""
	for i in cmd:
		command += str(ord(i)) + " "
	print command.rstrip()
	connection.write(cmd)

def get_sensor_values():
	#Sensor Packet ID Structure: [149] n [p0] [p1] ... [pn]
	#	where n is the number of packets requested
	#
	# Order used here:
	#	Packet ID - packet type 			[data format]
	# ----------------------------------------------------------
	#		   43 - left encoder counts 	[2 Bytes, signed]
	#		   44 - right encoder counts 	[2 Bytes, signed]
	#		    7 - bumps and wheel dropes 	[1 Byte, unsigned]
	#		   21 - charging state			[1 Byte, unsigned]
	#		   22 - battery voltage			[2 Bytes, unsigned]
	#		   25 - battery charge			[2 Bytes, unsigned]
	#		   26 - battery capacity		[2 Bytes, unsigned]
	#		   35 - OI mode 				[1 Byte, unsigned]
	charging_state = [False]*6
	mode = [False]*4
	global connection
	with lock:
		connection.reset_input_buffer()
		send_command('149 8 43 44 7 21 22 25 26 35')
		data = struct.unpack('>hhBBHHHB', connection.read(13))
	left = int(data[0])
	right = int(data[1])
	try:
		if int(data[3]) > 5 or int(data[3])<0:
			charging_state[0] = True
		else:
			charging_state[int(data[3])] = True
		mode[int(data[7])] = True
		bumpers = list(map(lambda x: bool(int(x)), list(get_bin(data[2],4))))
		sensors.update(dict(zip(group['bumpers'],bumpers)))
		sensors.update(dict(zip(group['encoders'],[data[0],data[1]])))
		sensors.update(dict(zip(group['charging_state'],charging_state)))
		sensors.update(dict(zip(group['battery'],data[4:7])))
		sensors.update(dict(zip(group['mode'],mode)))
	except:
		pass
	#print sensors
	return (left,right)

def drive_pwm(pwm_left,pwm_right):
	global connection
	cmd = struct.pack(">Bhh", 146, pwm_right, pwm_left)
	connection.write(cmd)
# When switching from Full mode -> Passive Mode -> Full Mode, Encoders increment by two

send_command('7')
time.sleep(6)
send_command('128')
time.sleep(0.1)
send_command('132')
time.sleep(0.5)

# Initialize sensor states
(i1,i2) = get_sensor_values()

subscriber_thread = threading.Thread(target=subscribers)
subscriber_thread.daemon = True
subscriber_thread.start()

high_freq_pub_thread = threading.Thread(target=high_frequency_publishers)
high_freq_pub_thread.daemon = True
high_freq_pub_thread.start()

low_freq_pub_thread = threading.Thread(target=low_frequency_publishers)
low_freq_pub_thread.daemon = True
low_freq_pub_thread.start()

old_time = time.time()

(old_left,old_right) = get_sensor_values()
start_left = old_left
start_right = old_right

cur_v = 0

vl = 0
vr = 0

max_dPWM = 50

cur_pwm_left = 0
cur_pwm_right = 0

l = 0.235
#dt = 0.1
dt = 0.05

time.sleep(dt)

i = 0

threshold = 5

x = 0.0
y = 0.0
theta = 0.0

while subscriber_thread.is_alive():

	if move_request == False and rotate_request == False:

		clear_for_service = False

		if set_v == 0:
			cur_v = 0
		if set_v>0 and cur_v<0:
			cur_v = 0
		if set_v<0 and cur_v>0:
			cur_v = 0
		if i % 5 == 0:
		#if cur_v < set_v and vl >= cur_v and vr >= cur_v and abs(vl-vr) <= threshold:
			if cur_v < set_v:
				cur_v += min(20, int((set_v-cur_v)))
			if cur_v > set_v:
				cur_v += max(-20, int((set_v-cur_v)))
		v = cur_v
		#print "-----"
		new_time = time.time()
		(left,right) = get_sensor_values()
		if not linear:
			start_left = left
			start_right = right
		
		#print "Left: " + str(left) + " Right: " + str(right)

		d_ticks_l = left - old_left
		d_ticks_r = right - old_right

		# correct for rollover
		if abs(d_ticks_l) > 32767:
			start_left = left
			start_right = right
			if d_ticks_l > 0:
				d_ticks_l += -65535
			if d_ticks_l < 0:
				d_ticks_l += 65535
		if abs(d_ticks_r) > 32767:
			start_left = left
			start_right = right
			if d_ticks_r > 0:
				d_ticks_r += -65535
			if d_ticks_r < 0:
				d_ticks_r += 65535

		average_ticks = ((left-start_left) + (right-start_right))*0.5

		d_l_avg = int(round(average_ticks - left + start_left))
		d_r_avg = int(round(average_ticks - right + start_right))

		dpwm_rel_l = min(abs(d_l_avg),10)*sign(d_l_avg)
		dpwm_rel_r = min(abs(d_r_avg),10)*sign(d_r_avg)

		#print "dLeft: " + str(d_ticks_l) + " dRight: " + str(d_ticks_r)
		dl = d_ticks_l * mm_per_tick
		dr = d_ticks_r * mm_per_tick

		d_center = 0.001*(dl+dr)*0.5
		phi = ((dr-dl)*0.001)/l
		theta += phi
		x += d_center*cos(theta)
		y += d_center*sin(theta)
		broadcast_tf(x,y,theta)

		#print "dL(mm): %.2f dR(mm): %.2f" % (dl,dr)
		dtime = new_time - old_time
		vl = dl/dtime
		vr = dr/dtime
		#print "vL(mm/s): %.2f vR(mm/s): %.2f" % (vl,vr)
		l_error = (l_const*v) - vl
		r_error = (r_const*v) - vr

		dPWM_l = int(l_error*0.5)
		dPWM_r = int(r_error*0.5)

		if abs(dPWM_l) > max_dPWM:
			dPWM_l = max_dPWM * sign(dPWM_l)
		if abs(dPWM_r) > max_dPWM:
			dPWM_r = max_dPWM * sign(dPWM_r)

		cur_pwm_left += dPWM_l
		cur_pwm_right += dPWM_r
		
		if linear:
			cur_pwm_left += dpwm_rel_l
			cur_pwm_right += dpwm_rel_r

		if cur_pwm_left > 255:
			cur_pwm_left = 255
		if cur_pwm_left < -255:
			cur_pwm_left = -255

		if cur_pwm_right > 255:
			cur_pwm_right = 255
		if cur_pwm_right < -255:
			cur_pwm_right = -255

		if v == 0 and vl == 0 and vr == 0:
			cur_pwm_left = 0
			cur_pwm_right = 0

		drive_pwm(cur_pwm_left,cur_pwm_right)
		#print "Lpwm: " + str(cur_pwm_left) + " Rpwm: " + str(cur_pwm_right)
		old_left = left
		old_right = right
		old_time = new_time
		#time.sleep(dt)
		i += 1

	if (rotate_request or move_request) and not clear_for_service:
		drive_direct_command(0,0)
		time.sleep(0.5)
		clear_for_service = True

	time.sleep(dt)

drive_direct_command(0,0)
time.sleep(0.5)
send_command('128')
