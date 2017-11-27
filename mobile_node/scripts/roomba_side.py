import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import time


ser = serial.Serial();
ser.baudrate = 57600
ser.port = '/dev/ttyUSB1'
ser.bytesize = 8
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.open()

cts = True

def send_serial(message_to_send):
	time.sleep(0.013)
	if ser.isOpen():
		ser.write(message_to_send.encode())
        #print message_to_send
    else:
    	ser.open()
    	ser.write(message_to_send.encode())
    

def send(data):
    header = data.header;
    child_frame_id = " "+data.child_frame_id
    pose = data.pose.pose
    twist = data.twist.twist
    identifier = "0"
    header_msg = " "+str(header.seq)+" "+str(header.stamp.secs)+" "+str(header.stamp.nsecs)+" "+header.frame_id
    pose_msg = " "+str(pose.position.x)+" "+str(pose.position.y)+" "+str(pose.position.z)+" "+str(pose.orientation.x)+" "+str(pose.position.y)+" "+str(pose.position.z)
    twist_msg = " "+str(twist.linear.x)+" "+str(twist.linear.x)+" "+str(twist.linear.y)+" "+str(twist.linear.z)+" "+str(twist.angular.x)+" "+str(twist.angular.y)+" "+str(twist.angular.z)
    final_msg = identifier+header_msg+child_frame_id+pose_msg+twist_msg+"\r\n"
    #print final_msg
    send_serial(final_msg)

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
	print "sending"
	pub.publish(base_vel)
	rate.sleep()

def listen():
	response = None
	if ser.isOpen():
		response = ser.readline()
	else:
		ser.open()
		response = ser.readline()

	if response == None:
		return
	else:
		print "listening"
		msg = response.split()
		header = msg[0]
		if header == 1:
			pub_cmd(msg)
		else:
			return

def odom_callback(data):
	global cts

	if cts == True:
		#send odom data over serial
		send(data)
		cts = False
		#time.sleep(0.5)
		#cts = True
	else:
		#listen to incoming message
		listen()
		cts = True
		#time.sleep(0.5)
		#cts.False	

def listener():

    rospy.init_node('ros_to_string', anonymous=True)

    rospy.Subscriber('RosAria/pose', Odometry, odom_callback)
        
    rospy.spin()

if __name__ == '__main__':
    listener()