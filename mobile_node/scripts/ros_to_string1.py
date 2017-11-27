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

cmd_message = ""
odom_message = ""
odom_flag = False
cmd_flag = False

def send_msg(message_to_send):
    time.sleep(0.02)
    if(ser.isOpen()): 
        print message_to_send
        ser.write(message_to_send.encode())
    else:
        print "was closed"
        ser.open()
        ser.write(message_to_send.encode())

def cmd_vel_callback(data):
    linear = [round(data.linear.x,2), round(data.linear.y,2), round(data.linear.z,2)]
    angular = [round(data.angular.x,2), round(data.angular.y,2), round(data.angular.z,2)]

    lin_x = str(linear[0])
    lin_y = str(linear[1])
    lin_z = str(linear[2])
    ang_x = str(angular[0])
    ang_y = str(angular[1])
    ang_z = str(angular[2])
    global cmd_message
    cmd_message = "1 linear.x "+lin_x+" linear.y "+lin_y+" linear.z "+lin_z+" angular.x "+ang_x+" angular.y "+ang_y+" angular.z "+ang_z+"\r\n"
    send_msg(cmd_message)
    #global cmd_flag
    #cmd_flag = True

    #print message
    

def odom_callback(data):
    header = data.header;
    child_frame_id = data.child_frame_id
    pose = data.pose.pose
    twist = data.twist.twist
    identifier = "0"
    header_msg = " "+str(header.seq)+" "+str(header.stamp.secs)+" "+str(header.stamp.nsecs)+" "+header.frame_id
    pose_msg = " "+str(pose.position.x)+" "+str(pose.position.y)+" "+str(pose.position.z)+" "+str(pose.orientation.x)+" "+str(pose.position.y)+" "+str(pose.position.z)
    twist_msg = " "+str(twist.linear.x)+" "+str(twist.linear.x)+" "+str(twist.linear.y)+" "+str(twist.linear.z)+" "+str(twist.angular.x)+" "+str(twist.angular.y)+" "+str(twist.angular.z)
    global odom_message
    odom_message = identifier+header_msg+child_frame_id+pose_msg+twist_msg+"\r\n"
    send_msg(odom_message)
    #global odom_flag
    #odom_flag = True
    #print final_msg
    #send_msg(final_msg) 

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
