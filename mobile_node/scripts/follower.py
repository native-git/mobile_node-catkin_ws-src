#!/usr/bin/env python  

#import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import time
from numpy import interp
from numpy import sign
#import turtlesim.srv

"""
def getNewVelocities():
    try:
        (trans,rot) = listener.lookupTransform('/target', '/usb_cam2_new', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        raise e

    linear = math.sqrt(trans[0] ** 2 + trans[1] ** 2)   #0.1
    angular = math.atan2(trans[1], trans[0])            #0.5

    return [linear, angular]
"""
if __name__ == '__main__':
    rospy.init_node('follower')

    listener = tf.TransformListener()

    #rospy.wait_for_service('spawn')
    #spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    #spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('/RosAria/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        vel = list()

        try:
            #time.sleep(2)
            #now = rospy.Time.now()
            #listener.waitForTransform('/target', '/usb_cam2_new', rospy.Time.now(), rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform('/robot', '/target', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue 

        print "LINEAR ______________________"
        

        cmd = geometry_msgs.msg.Twist()
        angular = math.atan2(trans[1], trans[0])
        linear = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        if linear > 3:
            distance = 3
        else:
            distance = linear

        lin_speed = interp(distance, [0,3], [0,0.3])
        rot_speed = sign(angular) * interp(abs(angular), [0, 3.142], [0, 0.5])
        
        if rot_speed < 0.01:
            cmd.linear.x = lin_speed

        print rot_speed
        '''
        if trans[1] < 0.0:
            angular = -1 * math.atan2(trans[1], trans[0])            #0.5
        else:
            angular = math.atan2(trans[1], trans[0])            
        '''
        linear = math.sqrt(trans[0] ** 2 + trans[1] ** 2)   #0.1
        
        '''
        if linear < 0.2:
            linear = 0.0
            angular = 0.0
        '''
        #print linear
        print "ANGULAR ______________________"
        print angular
        
        #cmd.linear.x = 0.0 * linear
        cmd.angular.z = rot_speed

        turtle_vel.publish(cmd)
        

        rate.sleep()    #pub ang




'''
        while angular > 0.01:
            #linear = 0.0
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * angular
            turtle_vel.publish(cmd)
            print angular
            vel = getNewVelocities()
            linear = vel[0]
            angular = vel[1]
            rate.sleep()    #pub ang

        while linear > 0.1:
            #angular = 0.0
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = 0.1 * linear
            cmd.angular.z = 0.0
            turtle_vel.publish(cmd)
            print linear
            vel = getNewVelocities()
            linear = vel[0]
            angular = 0.0
            rate.sleep()    #pub lin            
'''
