import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import time

ser = serial.Serial();
ser.baudrate = 57600
ser.port = '/dev/ttyUSB0'
ser.bytesize = 8
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.open()

cts = False

rospy.init_node('string_to_ros', anonymous=True)

def send_serial(message_to_send):
	time.sleep(0.013)
	if ser.isOpen():
		ser.write(message_to_send.encode())
	else:
		ser.open()
		ser.write(message_to_send.encode())

def send(data):
	print "Sending"
	linear = [data.linear.x, data.linear.y, data.linear.z]
	angular = [data.angular.x, data.angular.y, data.angular.z]

	lin_x = str(data.linear.x)
	lin_y = str(data.linear.y)
	lin_z = str(data.linear.z)
	ang_x = str(data.angular.x)
	ang_y = str(data.angular.y)
	ang_z = str(data.angular.z)
    
	cmd_message = "1 "+lin_x+" "+lin_y+" "+lin_z+" "+ang_x+" "+ang_y+" "+ang_z+"\r\n"
	print cmd_message
	send_serial(cmd_message)

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

	#print pub_odom
	print "publishing"
	pub1.publish(pub_odom)
	rate.sleep()

def listen():
	print "inside listen"
	response = None

	if ser.isOpen():
		response = ser.readline()
		print response
	else:
		ser.open()
		response = ser.readline()
		print response

	if response == None:
		#print "returning"
		return
	else:
		print "listening"
		msg = response.split()
		header = msg[0]
		if header == "0":
			pub_odometry(msg)
		else:
			return		
	return

def cmd_vel_callback(data):
	global cts
	print "inside cmd_vel callback"
	if cts == True:
		#send cmd_vel data over serial
		send(data)
		cts = False
		print cts
		#time.sleep(0.5)
		#cts = True
	else:
		#listen to incoming message
		listen()
		cts = True
		print cts
		#time.sleep(0.5)
		#cts = False

def listener():

    #rospy.init_node('ros_to_string', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
        
    rospy.spin()

if __name__ == '__main__':
    listener()
