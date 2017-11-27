#!/usr/bin/env python  
#import roslib
#roslib.load_manifest('learning_tf')
import rospy
import argparse
import tf
#import turtlesim.msg

def handle_turtle_pose(x, y, theta, turtlename):

    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     turtlename,
                     "map")

def Main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", help="position - X", type=float)
    parser.add_argument("y", help="position - Y", type=float)
    parser.add_argument("theta", help="position - Theta", type=float)

    args = parser.parse_args()

    while not rospy.is_shutdown():
        handle_turtle_pose(args.x, args.y, args.theta, "target")
    


if __name__ == '__main__':
    rospy.init_node('target')
    rospy.Rate(10.0)
    #turtlename = rospy.get_param('~turtle')
    
    Main()