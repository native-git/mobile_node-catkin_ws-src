import rospy
from nav_msgs.msg import Odometry

flag = False

def odom_callback(data):
	global flag
	if flag == True:
		print "flag was true"
		flag = False
	else:
		print "flag was false"
		flag = True


def listener():

    rospy.init_node('ros_to_string', anonymous=True)

    rospy.Subscriber('RosAria/pose', Odometry, odom_callback)    
    rospy.spin()

if __name__ == '__main__':
    listener()