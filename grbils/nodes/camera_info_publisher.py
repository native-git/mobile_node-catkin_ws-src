#! /usr/bin/env python2

import time
from sensor_msgs.msg import Image
import yaml
from sensor_msgs.msg import CameraInfo
import rospy

# initialize values
update_requested = False

rospy.init_node('frame_to_video_publisher')

info_pub = rospy.Publisher("/phone/camera/camera_info", CameraInfo, queue_size=10)

rate = rospy.Rate(10)

def retrieve_image():

	global image_msg

	got_one = False

	while not got_one:
		try:
			image_msg = rospy.wait_for_message("/phone/camera/image_raw", Image)
			got_one = True
		except:
			(rospy.exceptions.ROSException,rospy.exceptions.ROSInterruptException)
			continue
		rate.sleep()

def yaml_to_CameraInfo(yaml_fname):

	# Load data from file
	with open(yaml_fname, "r") as file_handle:
		calib_data = yaml.load(file_handle)
	# Parse
	camera_info_msg = CameraInfo()
	camera_info_msg.header.frame_id = calib_data["camera_name"]
	camera_info_msg.width = calib_data["image_width"]
	camera_info_msg.height = calib_data["image_height"]
	camera_info_msg.K = calib_data["camera_matrix"]["data"]
	camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
	camera_info_msg.R = calib_data["rectification_matrix"]["data"]
	camera_info_msg.P = calib_data["projection_matrix"]["data"]
	camera_info_msg.distortion_model = calib_data["distortion_model"]
	return camera_info_msg

if __name__ == '__main__':

	# Filename is the absolute path to the camera_info yaml
	filename = "/home/native/.ros/camera_info/camera.yaml"

	# Parse yaml file
	camera_info_msg = yaml_to_CameraInfo(filename)

	while not rospy.is_shutdown():

		retrieve_image()

		camera_info_msg.header.stamp = image_msg.header.stamp
		info_pub.publish(camera_info_msg)
		rate.sleep()