#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import os

ser = serial.Serial();
ser.baudrate = 57600
ser.port = '/dev/ttyUSB0'
ser.bytesize = 8
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
#ser.open()
def send_msg(message):
    try:
        ser.open()
        print "Connected usb0"
        ser.write(message.encode())
    except serial.serialutil.SerialException:
        print "Cannot Connect to usb0"
        ser.close()
    #ser.write(message.encode())

def msg_creator_cmd_vel(linear, angular):
    lin_x = str(linear[0])
    lin_y = str(linear[1])
    lin_z = str(linear[2])
    ang_x = str(angular[0])
    ang_y = str(angular[1])
    ang_z = str(angular[2])
    message = "1 linear.x "+lin_x+" linear.y "+lin_y+" linear.z "+lin_z+" angular.x "+ang_x+" angular.y "+ang_y+" angular.z "+ang_z+" \n"
    return message

def cmd_vel_callback(data):
    linear = [data.linear.x, data.linear.y, data.linear.z]
    angular = [data.angular.x, data.angular.y, data.angular.z]
        
    #if is_connected()
    message = msg_creator_cmd_vel(linear, angular)
    print message
    send_msg(message)

def odom_callback(data):
    header = data.header;
    child_frame_id = data.child_frame_id
    pose = data.pose.pose
    twist = data.twist.twist
    identifier = "0"
    header_msg = " "+str(header.seq)+" "+str(header.stamp.secs)+" "+str(header.stamp.nsecs)+" "+header.frame_id
    pose_msg = " "+str(pose.position.x)+" "+str(pose.position.y)+" "+str(pose.position.z)+" "+str(pose.orientation.x)+" "+str(pose.position.y)+" "+str(pose.position.z)
    twist_msg = " "+str(twist.linear.x)+" "+str(twist.linear.x)+" "+str(twist.linear.y)+" "+str(twist.linear.z)+" "+str(twist.angular.x)+" "+str(twist.angular.y)+" "+str(twist.angular.z)
    final_msg = identifier+header_msg+child_frame_id+pose_msg+twist_msg+"\n"
    #print final_msg
    send_msg(final_msg) 

def listener():

    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ros_to_string', anonymous=True)

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
