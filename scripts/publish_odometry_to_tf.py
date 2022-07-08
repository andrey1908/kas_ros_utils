#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-odom', '--odom-topic', required=True, type=str)
    return parser


def publish_odometry_to_tf(odom: Odometry):
    global tf_broadcaster

    transform = TransformStamped()
    transform.header = odom.header
    transform.child_frame_id = odom.child_frame_id
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.translation.z = odom.pose.pose.position.z
    transform.transform.rotation = odom.pose.pose.orientation

    tf_broadcaster.sendTransform(transform)


if __name__ == '__main__':
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    odom_topic = args.odom_topic
    
    rospy.init_node('publish_odometry_to_tf')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber(odom_topic, Odometry, publish_odometry_to_tf)
    rospy.spin()
    
