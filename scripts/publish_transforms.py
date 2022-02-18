#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-odom', '--odom-topic', required=True, type=str)
    return parser


def publish_transform(odom: Odometry):
    global tf_broadcaster

    transform = TransformStamped()
    transform.header = odom.header
    transform.child_frame_id = odom.child_frame_id
    transform.transform.translation = odom.pose.pose.position
    transform.transform.rotation = odom.pose.pose.orientation

    tf_broadcaster.sendTransform(transform)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    odom_topic = args.odom_topic
    
    rospy.init_node('publish_transforms')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber(odom_topic, Odometry, publish_transform)
    rospy.spin()
    
