#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import argparse
import numpy as np
from ros_numpy.geometry import transform_to_numpy, numpy_to_transform, pose_to_numpy, numpy_to_pose


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-odom', '--odom-topic', required=True, type=str)
    parser.add_argument('--new-odom-frame-name', required=True, type=str)
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


def twist_to_numpy(twist):
    arr = np.zeros((4, 4))
    arr[:3, :3] = vector_to_skew([twist.angular.x, twist.angular.y, twist.angular.z])
    arr[:3, 3] = [twist.linear.x, twist.linear.y, twist.linear.z]
    arr[3, 3] = 1
    return arr


def numpy_to_twist(arr):
    twist = Twist()
    w = skew_to_vector(arr[:3, :3])
    twist.linear.x, twist.linear.y, twist.linear.z = arr[:3, 3]
    twist.angular.x, twist.angular.y, twist.angular.z = w
    return twist


def odom_received(odom):
    global new_odom_frame_name
    global new_child_frame
    global publish_odometry_transforms
    global drift_vector

    global tfBroadcaster
    global tfBuffer
    global odom_publisher
    global first_stamp

    ros_transform = tfBuffer.lookup_transform(new_child_frame, odom.child_frame_id, odom.header.stamp)

    transform = transform_to_numpy(ros_transform.transform)
    transform_r4 = np.eye(4)
    transform_r4[:3, :3] = transform[:3, :3]

    pose = pose_to_numpy(odom.pose.pose)
    twist = twist_to_numpy(odom.twist.twist)

    if first_stamp == rospy.Time(0):
        first_stamp = odom.header.stamp
    pose[:3, 3] += drift_vector * (odom.header.stamp - first_stamp).to_sec()
    twist[:3, 3] += np.matmul(np.linalg.inv(pose[:3, :3]), drift_vector)

    new_pose = np.matmul(np.matmul(transform, pose), np.linalg.inv(transform))
    new_twist = np.matmul(np.matmul(transform_r4, twist), np.linalg.inv(transform))

    new_ros_pose = numpy_to_pose(new_pose)
    new_ros_twist = numpy_to_twist(new_twist)

    if odom_publisher is not None:
        new_odom = Odometry()
        new_odom.header.stamp = odom.header.stamp
        new_odom.header.frame_id = new_odom_frame_name
        new_odom.child_frame_id = new_child_frame
        new_odom.pose.pose = new_ros_pose
        new_odom.twist.twist = new_ros_twist
        odom_publisher.publish(new_odom)

    if publish_odometry_transforms:
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = new_odom_frame_name
        transform.child_frame_id = new_child_frame
        transform.transform = numpy_to_transform(new_ros_pose)
        tfBroadcaster.sendTransform(transform)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    odom_topic = args.odom_topic
    new_odom_frame_name = args.new_odom_frame_name
    new_child_frame = args.new_child_frame
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
    
