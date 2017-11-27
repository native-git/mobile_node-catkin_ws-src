#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg  import Twist
import time

rospy.init_node("initialize_create_2")
velocity_publisher = rospy.Publisher('roomba3/cmd_vel', Twist, queue_size=10)
mode_publisher = rospy.Publisher("roomba3/mode", Int16, queue_size=10)

def switch_to_full_mode():
	mode = Int16()
	mode.data = 3
	mode_publisher.publish(mode)

def drive_backwards():
	vel_msg = Twist()
	vel_msg.linear.x = -0.25
	velocity_publisher.publish(vel_msg)
	time.sleep(3)
	vel_msg = Twist()
	vel_msg.linear.x = 0.0
	velocity_publisher.publish(vel_msg)

switch_to_full_mode()
time.sleep(2)
drive_backwards()