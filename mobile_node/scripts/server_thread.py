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
ser.port = '/dev/ttyUSB0'
ser.bytesize = 8
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.open()				## Open Serial Port

rospy.init_node('server_thread', anonymous=True)		## init ros node

######### Queues ##########3
ros_q = Queue(maxsize=0)
serial_q = Queue(maxsize=0)

########### Initial cmd_vel #################
init_cmd_vel = Twist()
init_cmd_vel.linear.x = 0.0
init_cmd_vel.linear.y = 0.0
init_cmd_vel.linear.z = 0.0
init_cmd_vel.angular.x = 0.0
init_cmd_vel.angular.y = 0.0
init_cmd_vel.angular.z = 0.0

def fetch_serial():
	print "ser thread"
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
			if header == "0" and serial_q.full() == False:
				serial_q.put_nowait(msg)

def cmd_vel_callback(data):
	global init_cmd_vel
	if init_cmd_vel == data:
		return
	init_cmd_vel = data
	lin_x = str(data.linear.x)
	lin_y = str(data.linear.y)
	lin_z = str(data.linear.z)
	ang_x = str(data.angular.x)
	ang_y = str(data.angular.y)
	ang_z = str(data.angular.z)
	print "inside callback"
 	cmd_message = "1 "+lin_x+" "+lin_y+" "+lin_z+" "+ang_x+" "+ang_y+" "+ang_z+"\r\n"
	if ros_q.full() == False:
		ros_q.put_nowait(cmd_message)
	
def send_serial(message_to_send):
	#time.sleep(0.013)

	if ser.isOpen():
		ser.write(message_to_send.encode())
		print message_to_send
	else:
		ser.open()
		ser.write(message_to_send.encode())
		print message_to_send

def pub_odometry(data):
	pub1 = rospy.Publisher('odom', Odometry, queue_size = 1)
	#rospy.init_node('string_to_ros', anonymous=True)
	rate = rospy.Rate(10) # 10hz		
	pub_odom = Odometry()	

	pub_odom.header.seq = int(data[1])
	pub_odom.header.stamp.secs = int(data[2])
	pub_odom.header.stamp.nsecs = int(data[3])
	pub_odom.header.frame_id = data[4]

	pub_odom.child_frame_id = data[5]

	pub_odom.pose.pose.position.x = float(data[6])
	pub_odom.pose.pose.position.y = float(data[7])
	pub_odom.pose.pose.position.z = float(data[8])
	pub_odom.pose.pose.orientation.x = float(data[9])
	pub_odom.pose.pose.orientation.y = float(data[10])
	pub_odom.pose.pose.orientation.z = float(data[11])

	pub_odom.twist.twist.linear.x = float(data[12])
	pub_odom.twist.twist.linear.y = float(data[13])
	pub_odom.twist.twist.linear.z = float(data[14])
	pub_odom.twist.twist.angular.x = float(data[15])
	pub_odom.twist.twist.angular.y = float(data[16])
	pub_odom.twist.twist.angular.z = float(data[17])

	print data
	print "publishing"
	pub1.publish(pub_odom)
	#rate.sleep()

def ros_subscriber():
	print "ros thread"
	while True:
		rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

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
			pub_odometry(serial_msg)


if __name__ == '__main__':
	Main()