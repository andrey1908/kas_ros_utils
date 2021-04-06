#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import sys
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', required=True, type=str)
    parser.add_argument('--target-frame', required=True, type=str)
    return parser


def odom_received(odom):
    global target_frame

    global tfBroadcaster
    global tfBuffer

    odom_trans = geometry_msgs.msg.TransformStamped()
    odom_trans.header = odom.header
    odom_trans.child_frame_id = target_frame
    odom_trans.transform.translation = odom.pose.pose.position
    odom_trans.transform.rotation = odom.pose.pose.orientation
    tfBroadcaster.sendTransform(odom_trans)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    odom_topic = args.odom_topic
    target_frame = args.target_frame
    
    rospy.init_node('publish_odometry_transformation')
    tfBroadcaster = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.Subscriber(odom_topic, Odometry, odom_received)
    rospy.spin()
    
