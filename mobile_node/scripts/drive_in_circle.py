import rospy
from geometry_msgs.msg import Twist


def Main():
	msg = Twist()
	while not rospy.is_shutdown():
		msg.linear.x = 0.05
		msg.angular.z = 0.15
		pub.publish(msg)

	msg.linear.x = 0.0
	msg.linear.y = 0.0
	pub.publish(msg)


if __name__ == '__main__':

	rospy.init_node('circle_driver')
	pub = rospy.Publisher("RosAria/cmd_vel", Twist, queue_size = 10)
	r = rospy.Rate(100)
	Main()

