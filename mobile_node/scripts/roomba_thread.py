import rospy
import serial
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from threading import Thread
from multiprocessing import Queue
from Queue import *

###### Serial Port initialization #######
ser = serial.Serial();
ser.baudrate = 57600
ser.port = '/dev/ttyUSB1'
ser.bytesize = 8
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.open()				## Open Serial Port

rospy.init_node('roomba_thread', anonymous=True)		## init ros node

######### Queues ##########3
ros_q = Queue(maxsize=2)
serial_q = Queue(maxsize=2)


init_odom = Odometry()
init_odom.pose.pose.position.x = 0.0
init_odom.pose.pose.position.y = 0.0
init_odom.pose.pose.position.z = 0.0
init_odom.twist.twist.linear.x = 0.0
init_odom.twist.twist.linear.y = 0.0
init_odom.twist.twist.linear.z = 0.0
init_odom.twist.twist.angular.x = 0.0
init_odom.twist.twist.angular.y = 0.0
init_odom.twist.twist.angular.z = 0.0

def fetch_serial():
	while True:
		response = None
		if ser.isOpen():
			response = ser.readline()
			#print response
		else:
			ser.open()
			response = ser.readline()
			#print response

		if response != None:
			print "listening"
			msg = response.split()
			header = msg[0]
			if header == "1" and serial_q.full() == False:
				serial_q.put_nowait(msg)


def odom_callback(data):
	
	global init_odom
	if init_odom.pose.pose.position == data.pose.pose.position and init_odom.twist.twist == data.twist.twist:
		return
	init_odom = data
	header = data.header;
	child_frame_id = " "+data.child_frame_id
	pose = data.pose.pose
	twist = data.twist.twist
	identifier = "0"
	header_msg = " "+str(header.seq)+" "+str(header.stamp.secs)+" "+str(header.stamp.nsecs)+" "+header.frame_id
	pose_msg = " "+str(pose.position.x)+" "+str(pose.position.y)+" "+str(pose.position.z)+" "+str(pose.orientation.x)+" "+str(pose.position.y)+" "+str(pose.position.z)
	twist_msg = " "+str(twist.linear.x)+" "+str(twist.linear.x)+" "+str(twist.linear.y)+" "+str(twist.linear.z)+" "+str(twist.angular.x)+" "+str(twist.angular.y)+" "+str(twist.angular.z)
	final_msg = identifier+header_msg+child_frame_id+pose_msg+twist_msg+"\r\n"
	if ros_q.full() == False:
		ros_q.put_nowait(final_msg)
	return

def send_serial(message_to_send):
	#time.sleep(0.013)
	if ser.isOpen():
		ser.write(message_to_send.encode())
	else:
		ser.open()
		ser.write(message_to_send.encode())

def pub_cmd(data):
	pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size = 1)

	rate = rospy.Rate(10) # 10hz
	base_vel = Twist()
	
	base_vel.linear.x = float(data[1])
	base_vel.linear.y = float(data[2])
	base_vel.linear.z = float(data[3])
	base_vel.angular.x = float(data[4])
	base_vel.angular.y = float(data[5])
	base_vel.angular.z = float(data[6])
	print "sending"
	pub.publish(base_vel)
	rate.sleep()

def ros_subscriber():
	while True:
		time.sleep(2)	
		rospy.Subscriber('RosAria/pose', Odometry, odom_callback)

def Main():
	#thread1 = Thread(target=fetch_ros, args=("Thread2", 2, 5))
	
	serial_thread = Thread(target=fetch_serial)
	serial_thread.start()

	ros_thread = Thread(target=ros_subscriber)
	ros_thread.start()

	while True:

		if ros_q.empty() == False:
			msg = ros_q.get_nowait()
			send_serial(msg)

		if serial_q.empty() == False:
			serial_msg = serial_q.get_nowait()
			pub_cmd(serial_msg)


if __name__ == '__main__':
	Main()