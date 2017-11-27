#!/usr/bin/env python  

import rospy
import math
import tf
import geometry_msgs.msg
import time
from numpy import interp
from numpy import sign

target = str()

def aim(target):
    while go:

        try:
            (trans,rot) = listener.lookupTransform('/robot', ('/' + (target)), rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print target

        angular = math.atan2(trans[1], trans[0])

        rot_speed = sign(angular) * interp(abs(angular), [0, 3.142], [0, 0.5])

        cmd.angular.z = rot_speed

        turtle_vel.publish(cmd)
                
        rate.sleep()

        go = False


if __name__ == '__main__':
    
    rospy.init_node('follower')
    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher('/RosAria/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    cmd = geometry_msgs.msg.Twist()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        targ = 'A'

        aim(targ)


    """
    mode = 1
    ## mode = 0 : go to A
    ## mode = 1 : go to target
    ## mode = 2 : stop


    while not rospy.is_shutdown():

        if mode == 0:
            print "------------------------------ MODE 0 --------------------------------------------------"
            try:
                (trans,rot) = listener.lookupTransform('/robot', '/A', rospy.Time.now())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 
            
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

            #print rot_speed

            cmd.angular.z = rot_speed

            print "----------- CMD ------------------------"
            
            print rot_speed
            print lin_speed
            turtle_vel.publish(cmd)
            
            rate.sleep()    #pub ang

            if distance < 0.02:
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                turtle_vel.publish(cmd)
                time.sleep(1)
                rate.sleep()
                mode = 1


        if mode == 1:
            print "------------------------------ MODE 1 --------------------------------------------------"
            try:
                (trans,rot) = listener.lookupTransform('/robot', '/target', rospy.Time.now())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 
            
            angular = math.atan2(trans[1], trans[0])
            linear = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            
            if linear > 3:
                distance = 3
            else:
                distance = linear

            lin_speed = interp(distance, [0,3], [0,0.3])
            rot_speed = sign(angular) * interp(abs(angular), [0, 3.142], [0, 0.5])

            cmd = geometry_msgs.msg.Twist()

            if rot_speed < 0.01:
                cmd.linear.x = lin_speed

            #print rot_speed

            cmd.angular.z = rot_speed

            print "----------- CMD ------------------------"
            print rot_speed
            print lin_speed
            turtle_vel.publish(cmd)
            
            rate.sleep()    #pub ang

            if distance < 0.02:
                cmd.angular.z = 0.0
                cmd.linear.x = 0.0
                turtle_vel.publish(cmd)
                time.sleep(1)
                rate.sleep()
                mode = 2


        if mode == 2:
            print "------------------------------ MODE 2 --------------------------------------------------"

            try:
                (trans,rot) = listener.lookupTransform('/robot', '/B', rospy.Time.now())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 
            
            angular = math.atan2(trans[1], trans[0])

            rot_speed = sign(angular) * interp(abs(angular), [0, 3.142], [0, 0.5])

            cmd = geometry_msgs.msg.Twist()

            #print rot_speed

            cmd.angular.z = rot_speed

            print "----------- CMD ------------------------"
            print rot_speed
            print lin_speed
            turtle_vel.publish(cmd)
            
            rate.sleep()

            """