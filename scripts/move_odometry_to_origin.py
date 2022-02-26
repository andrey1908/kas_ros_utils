#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import argparse
import numpy as np
from ros_numpy.geometry import pose_to_numpy, numpy_to_pose


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-odom', '--odom-topic', required=True, type=str)
    parser.add_argument('--new-odom-frame-name', required=True, type=str)
    parser.add_argument('-out-odom', '--out-odom-topic', required=True, type=str)
    return parser


def odom_received(odom):
    global new_child_frame
    global new_odom_frame_name
    global odom_publisher
    global first_pose_inv
    
    if (first_pose_inv is None):
        first_pose_inv = np.linalg.inv(pose_to_numpy(odom.pose.pose))

    pose = pose_to_numpy(odom.pose.pose)
    new_pose = np.matmul(first_pose_inv, pose)
    new_ros_pose = numpy_to_pose(new_pose)

    new_odom = Odometry()
    new_odom.header.stamp = odom.header.stamp
    new_odom.header.frame_id = new_odom_frame_name
    new_odom.child_frame_id = odom.child_frame_id
    new_odom.pose.pose = new_ros_pose
    new_odom.twist = odom.twist
    odom_publisher.publish(new_odom)


if __name__ == '__main__':
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    odom_topic = args.odom_topic
    new_odom_frame_name = args.new_odom_frame_name
    out_odom_topic = args.out_odom_topic
    
    rospy.init_node('odometry_to_origin')
    rospy.Subscriber(odom_topic, Odometry, odom_received)
    odom_publisher = rospy.Publisher(out_odom_topic, Odometry, queue_size=10)
    first_stamp = rospy.Time(0)
    first_pose_inv = None
    rospy.spin()
    
