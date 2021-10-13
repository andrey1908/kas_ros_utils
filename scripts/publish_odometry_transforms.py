#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import tf2_ros
import argparse
from copy import deepcopy


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-odom', '--odom-topic', required=True, type=str)
    return parser


def publish_odometry_transform(odom):
    global tfBroadcaster

    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    transform = TransformStamped(deepcopy(odom.header), odom.child_frame_id,
                                 Transform(Vector3(position.x, position.y, position.z), deepcopy(orientation)))
    tfBroadcaster.sendTransform(transform)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    odom_topic = args.odom_topic
    
    rospy.init_node('publish_odometry_transforms')
    tfBroadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber(odom_topic, Odometry, publish_odometry_transform)
    rospy.spin()
    
