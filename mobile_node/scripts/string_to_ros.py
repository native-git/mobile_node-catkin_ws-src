import serial
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ser1 = serial.Serial()
ser1.port = '/dev/ttyUSB1'
ser1.baudrate = 57600
ser1.bytesize = 8
ser1.parity = serial.PARITY_NONE
ser1.stopbits = serial.STOPBITS_ONE

#rospy.init_node('string_to_ros', anonymous=True)
#rate = rospy.Rate(10) # 10hz

def pub_cmd(data):
	pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size = 100)
	rospy.init_node('string_to_ros', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	base_vel = Twist()
	
	base_vel.linear.x = float(data[2])
	base_vel.linear.y = float(data[4])
	base_vel.linear.z = float(data[6])
	base_vel.angular.x = float(data[8])
	base_vel.angular.y = float(data[10])
	base_vel.angular.z = float(data[12])
	pub.publish(base_vel)
	rate.sleep()

def pub_odometry(data):
	pub1 = rospy.Publisher('odom', Odometry, queue_size = 100)
	rospy.init_node('string_to_ros', anonymous=True)
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

	pub1.publish(pub_odom)
	rate.sleep()

def listener():
	try:
		ser1.open()
		print "Connected usb1"
	except serial.serialutil.SerialException:
		print "Cannot Connect to usb0"
		ser1.close()

	while True:
		print "waiting for message"
		response = ser1.readline()
		if response != None:
			response = None
			msg = response.split()
			header = msg[0]
			if header == '0':
				pub_odometry(msg)			
				#print msg
				#response = None
			elif header == '1':
				pub_cmd(msg)
				#response = None

if __name__ == '__main__': 
    listener()			