#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
import tf2_ros
import argparse
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
from copy import deepcopy


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-odom', '--odom-topic', required=True, type=str)
    parser.add_argument('--new-odom-frame', required=True, type=str)
    parser.add_argument('--new-child-frame', required=True, type=str)
    parser.add_argument('-out-odom', '--out-odom-topic', type=str, default=None)
    parser.add_argument('-pub-tf', '--publish-odometry-transforms', action='store_true')
    parser.add_argument('-dv', '--drift-vector', type=float, nargs=3, default=[0., 0., 0.])
    return parser


def vector_to_skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])


def skew_to_vector(x):
    return np.array([x[2, 1], x[0, 2], x[1, 0]])


def ros_transform_to_matrix(ros_transform):
    transform = np.zeros((4, 4))
    rotation = ros_transform.transform.rotation
    transform[:3, :3] = quat2mat([rotation.w, rotation.x, rotation.y, rotation.z])
    translation = ros_transform.transform.translation
    transform[:3, 3] = [translation.x, translation.y, translation.z]
    transform[3, 3] = 1
    return transform


def ros_pose_to_matrix(ros_pose):
    pose = np.zeros((4, 4))
    pose[:3, :3] = quat2mat([ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z])
    pose[:3, 3] = [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z]
    pose[3, 3] = 1
    return pose


def ros_twist_to_matrix(ros_twist):
    twist = np.zeros((4, 4))
    twist[:3, :3] = vector_to_skew([ros_twist.angular.x, ros_twist.angular.y, ros_twist.angular.z])
    twist[:3, 3] = [ros_twist.linear.x, ros_twist.linear.y, ros_twist.linear.z]
    twist[3, 3] = 1
    return twist


def odom_received(odom):
    global new_child_frame
    global new_odom_frame
    global publish_odometry_transforms
    global drift_vector

    global tfBroadcaster
    global tfBuffer
    global odom_publisher
    global first_stamp

    ros_transform = tfBuffer.lookup_transform(new_child_frame, odom.child_frame_id, odom.header.stamp)

    transform = ros_transform_to_matrix(ros_transform)
    pose = ros_pose_to_matrix(odom.pose.pose)
    twist = ros_twist_to_matrix(odom.twist.twist)

    new_pose = transform @ pose @ np.linalg.inv(transform)
    new_position = new_pose[:3, 3]
    new_orientation = mat2quat(new_pose[:3, :3])

    new_twist = transform @ twist @ np.linalg.inv(transform)
    new_linear = new_twist[:3, 3]
    new_angular = skew_to_vector(new_twist[:3, :3])

    if first_stamp == rospy.Time(0):
        first_stamp = odom.header.stamp
    new_position += drift_vector * (odom.header.stamp - first_stamp).to_sec()
    new_linear += drift_vector

    if odom_publisher is not None:
        new_odom = Odometry(header=deepcopy(odom.header))
        new_odom.header.frame_id = new_odom_frame
        new_odom.child_frame_id = new_child_frame
        new_odom.pose = PoseWithCovariance(pose=Pose(Point(new_position[0], new_position[1], new_position[2]),
                                                     Quaternion(new_orientation[1], new_orientation[2], new_orientation[3], new_orientation[0])))
        new_odom.twist = TwistWithCovariance(twist=Twist(Vector3(new_linear[0], new_linear[1], new_linear[2]),
                                                         Vector3(new_angular[0], new_angular[1], new_angular[2])))
        odom_publisher.publish(new_odom)

    if publish_odometry_transforms:
        transform = TransformStamped(header=deepcopy(odom.header))
        transform.header.frame_id = new_odom_frame
        transform.child_frame_id = new_child_frame
        transform.transform = Transform(Vector3(new_position[0], new_position[1], new_position[2]),
                                        Quaternion(new_orientation[1], new_orientation[2], new_orientation[3], new_orientation[0]))
        tfBroadcaster.sendTransform(transform)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    odom_topic = args.odom_topic
    new_child_frame = args.new_child_frame
    new_odom_frame = args.new_odom_frame
    out_odom_topic = args.out_odom_topic
    publish_odometry_transforms = args.publish_odometry_transforms
    drift_vector = np.array(args.drift_vector)
    
    rospy.init_node('transform_odometry')
    tfBroadcaster = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.Subscriber(odom_topic, Odometry, odom_received)
    if out_odom_topic:
        odom_publisher = rospy.Publisher(out_odom_topic, Odometry, queue_size=10)
    else:
        odom_publisher = None
    first_stamp = rospy.Time(0)
    rospy.spin()
    
