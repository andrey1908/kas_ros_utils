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
    parser.add_argument('--in-odom-topic', required=True, type=str)
    parser.add_argument('--from-frame', required=True, type=str)
    parser.add_argument('--to-frame', required=True, type=str)
    parser.add_argument('--odom-frame', type=str, default='odom')
    parser.add_argument('--out-odom-topic', type=str, default=None)
    parser.add_argument('-pub-tf', '--publish-odom-tf', action='store_true')
    parser.add_argument('-df', '--drift-factor', type=float, default=0.)
    return parser


def odom_received(odom):
    global from_frame
    global to_frame
    global odom_frame
    global publish_odom_tf
    global drift_factor

    global tfBroadcaster
    global tfBuffer
    global odom_publisher
    global counter

    base_link_to_zed_left_camera_optical_frame = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time())

    translation = geometry_msgs.msg.Vector3Stamped()
    translation.header.frame_id = from_frame
    translation.vector = odom.pose.pose.position
    translation_base_link = tf2_geometry_msgs.do_transform_vector3(translation, base_link_to_zed_left_camera_optical_frame)

    rotation_vector = geometry_msgs.msg.Vector3Stamped()
    rotation_vector.header.frame_id = from_frame
    rotation_vector.vector.x = odom.pose.pose.orientation.x
    rotation_vector.vector.y = odom.pose.pose.orientation.y
    rotation_vector.vector.z = odom.pose.pose.orientation.z
    rotation_vector_base_link = tf2_geometry_msgs.do_transform_vector3(rotation_vector, base_link_to_zed_left_camera_optical_frame)

    odom_pose_base_link = geometry_msgs.msg.PoseStamped()
    odom_pose_base_link.pose.position = translation_base_link.vector
    odom_pose_base_link.pose.orientation.x = rotation_vector_base_link.vector.x
    odom_pose_base_link.pose.orientation.y = rotation_vector_base_link.vector.y
    odom_pose_base_link.pose.orientation.z = rotation_vector_base_link.vector.z
    odom_pose_base_link.pose.orientation.w = odom.pose.pose.orientation.w
    
    odom_pose_base_link.pose.position.y += counter * drift_factor
    counter += 1

    if publish_odom_tf:
        odom_to_base_link = geometry_msgs.msg.TransformStamped()
        odom_to_base_link.header.stamp = odom.header.stamp
        odom_to_base_link.header.frame_id = odom_frame
        odom_to_base_link.child_frame_id = to_frame
        odom_to_base_link.transform.translation = odom_pose_base_link.pose.position
        odom_to_base_link.transform.rotation = odom_pose_base_link.pose.orientation
        tfBroadcaster.sendTransform(odom_to_base_link)

    if odom_publisher is not None:
        odom_base_link = odom
        odom_base_link.header.frame_id = odom_frame
        odom_base_link.pose.pose = odom_pose_base_link.pose
        odom_publisher.publish(odom_base_link)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    in_odom_topic = args.in_odom_topic
    from_frame = args.from_frame
    to_frame = args.to_frame
    odom_frame = args.odom_frame
    out_odom_topic = args.out_odom_topic
    publish_odom_tf = args.publish_odom_tf
    drift_factor = args.drift_factor
    
    rospy.init_node('transform_odometry')
    tfBroadcaster = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.Subscriber(in_odom_topic, Odometry, odom_received)
    if out_odom_topic:
        odom_publisher = rospy.Publisher(out_odom_topic, Odometry, queue_size=10)
    else:
        odom_publisher = None
    counter = 0
    rospy.spin()
    
