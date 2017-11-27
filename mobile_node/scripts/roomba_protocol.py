import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import time
from multiprocessing import Queue
from Queue import *

ser = serial.Serial();
ser.baudrate = 57600
ser.port = '/dev/ttyUSB1'
ser.bytesize = 8
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.open()

rospy.init_node('ros_to_string', anonymous=True)

q = Queue(maxsize=1)
serial_q = Queue(maxsize=1)

def odom_callback(data):
	
	header = data.header;
	child_frame_id = " "+data.child_frame_id
	pose = data.pose.pose
	twist = data.twist.twist
	identifier = "0"
	header_msg = " "+str(header.seq)+" "+str(header.stamp.secs)+" "+str(header.stamp.nsecs)+" "+header.frame_id
	pose_msg = " "+str(pose.position.x)+" "+str(pose.position.y)+" "+str(pose.position.z)+" "+str(pose.orientation.x)+" "+str(pose.position.y)+" "+str(pose.position.z)
	twist_msg = " "+str(twist.linear.x)+" "+str(twist.linear.x)+" "+str(twist.linear.y)+" "+str(twist.linear.z)+" "+str(twist.angular.x)+" "+str(twist.angular.y)+" "+str(twist.angular.z)
	final_msg = identifier+header_msg+child_frame_id+pose_msg+twist_msg+"\r\n"
	if q.full() == False:
		q.put_nowait(final_msg)
	
	response = None
	if ser.isOpen():
		response = ser.readline()
		#print response
	else:
		ser.open()
		response = ser.readline()
		#print response

	if response != None:
		#print "returning"
		print "listening"
		msg = response.split()
		header = msg[0]
		
		if header == "1" and serial_q.full() == False:
			serial_q.put_nowait(msg)	

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

def send_serial(message_to_send):
	#time.sleep(0.013)
	if ser.isOpen():
		ser.write(message_to_send.encode())
	else:
		ser.open()
		ser.write(message_to_send.encode())


if __name__ == '__main__':
    while True:
    	rospy.Subscriber('RosAria/pose', Odometry, odom_callback)

    	if q.empty() == False:
    		msg = q.get_nowait()
    		send_serial(msg)

    	if serial_q.empty() == False:
    		serial_msg = serial_q.get_nowait()
    		pub_cmd(serial_msg)
    	else:
    		serial_msg = ["1", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0"]
    		pub_cmd(serial_msg)