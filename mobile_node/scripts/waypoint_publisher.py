#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher')

    listener = tf.TransformListener()

    publisher = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/map', '/target', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        ta = geometry_msgs.msg.TransformStamped()
        ta.header.frame_id = "/map"
        ta.header.stamp = rospy.Time.now()
        ta.child_frame_id = "/A"
        ta.transform.translation.x = trans[0]
        ta.transform.translation.y = (trans[1] - 0.25)
        ta.transform.translation.z = trans[2]

        ta.transform.rotation.x = rot[0]
        ta.transform.rotation.y = rot[1]
        ta.transform.rotation.z = rot[2]
        ta.transform.rotation.w = rot[3]

        tb = geometry_msgs.msg.TransformStamped()
        tb.header.frame_id = "/map"
        tb.header.stamp = rospy.Time.now()
        tb.child_frame_id = "/B"
        tb.transform.translation.x = trans[0]
        tb.transform.translation.y = (trans[1] + 0.5)
        tb.transform.translation.z = trans[2]

        tb.transform.rotation.x = rot[0]
        tb.transform.rotation.y = rot[1]
        tb.transform.rotation.z = rot[2]
        tb.transform.rotation.w = rot[3]

        tfma = tf.msg.tfMessage([ta])
        tfmb = tf.msg.tfMessage([tb])
        publisher.publish(tfma)
        publisher.publish(tfmb)

        rate.sleep()
#print "###############################"

            