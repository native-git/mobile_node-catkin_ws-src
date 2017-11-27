#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float64, Empty
import smtplib

ns = rospy.get_namespace()
print ns
warned = False

def send_alert(battery,data):
	message = "Subject: Low Battery Warning for: " + str(ns) + ""
	message += "Subsystem: " + str(battery) +" Level: "+ str(data)
	smtpObj = smtplib.SMTP('smtp.gmail.com',587)
	smtpObj.ehlo()
	smtpObj.starttls()
	smtpObj.login('nativeremote@gmail.com','native101')
	smtpObj.sendmail('nativeremote@gmail.com','mjs695@rutgers.edu',message)
	smtpObj.quit()

def battery_callback(data):
	global warned
	#print "Roomba Battery Ratio: " + str(data)
	if data.data <= 0.70 and warned == False:
		level = str(data.data*100) + " %% Capacity"
		send_alert("ROBOT_BATTERY", level)
		warned = True
	if data.data <= 0.65:
		rospy.set_param('keep_running', False)

def payload_battery_callback(data):
	global warned
	#print "Payload Battery Voltage: " + str(data)
	if data.data <= 12.9 and warned == False:
		level = str(data) + " Volts"
		send_alert("PAYLOAD_BATTERY", level)
		warned = True
	if data.data <= 12.9:
		rospy.set_param('keep_running', False)

def listener():
	rospy.init_node('battery_manager')
	rospy.Subscriber("battery/charge_ratio", Float32, battery_callback)
	rospy.Subscriber("payload_battery/voltage", Float64, payload_battery_callback)
	rospy.spin()

listener()