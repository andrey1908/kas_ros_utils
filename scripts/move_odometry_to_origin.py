#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import tf2_ros
import argparse
import numpy as np
from transforms3d.quaternions import mat2quat
from poses_handler import ros_message_to_matrix
from copy import deepcopy


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-odom', '--odom-topic', required=True, type=str)
    parser.add_argument('--new-odom-frame-name', required=True, type=str)
    parser.add_argument('-out-odom', '--out-odom-topic', required=True, type=str)
    return parser


def vector_to_skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])


def skew_to_vector(x):
    return np.array([x[2, 1], x[0, 2], x[1, 0]])


def ros_twist_to_matrix(ros_twist):
    twist = np.zeros((4, 4))
    twist[:3, :3] = vector_to_skew([ros_twist.angular.x, ros_twist.angular.y, ros_twist.angular.z])
    twist[:3, 3] = [ros_twist.linear.x, ros_twist.linear.y, ros_twist.linear.z]
    twist[3, 3] = 1
    return twist


def odom_received(odom):
    global new_child_frame
    global new_odom_frame_name
    global odom_publisher
    global first_pose_inv
    
    if (first_pose_inv is None):
        first_pose_inv = np.linalg.inv(ros_message_to_matrix(odom))

    pose = ros_message_to_matrix(odom)

    new_pose = first_pose_inv @ pose
    new_position = new_pose[:3, 3]
    new_orientation = mat2quat(new_pose[:3, :3])

    new_odom = Odometry(header=deepcopy(odom.header))
    new_odom.header.frame_id = new_odom_frame_name
    new_odom.pose = PoseWithCovariance(pose=Pose(Point(new_position[0], new_position[1], new_position[2]),
                                                 Quaternion(new_orientation[1], new_orientation[2], new_orientation[3], new_orientation[0])))
    odom_publisher.publish(new_odom)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    odom_topic = args.odom_topic
    new_odom_frame_name = args.new_odom_frame_name
    out_odom_topic = args.out_odom_topic
    
    rospy.init_node('odometry_to_origin')
    rospy.Subscriber(odom_topic, Odometry, odom_received)
    odom_publisher = rospy.Publisher(out_odom_topic, Odometry, queue_size=10)
    first_stamp = rospy.Time(0)
    first_pose_inv = None
    rospy.spin()
    
