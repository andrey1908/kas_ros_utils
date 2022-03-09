#!/usr/bin/env python
import rospy
import tf2_ros
import argparse
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array, array_to_pointcloud2
from ros_numpy.geometry import transform_to_numpy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from collections import deque
import time


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--point-cloud-topic', required=True, type=str)
    odometry_source = parser.add_mutually_exclusive_group(required=True)
    odometry_source.add_argument('--odometry-frame-id', type=str)
    odometry_source.add_argument('--odometry-topic', type=str)
    odometry_source.add_argument('--transforms-topic', type=str)
    parser.add_argument('-num', '--number-of-point-clouds-to-accumulate', required=True, type=int)
    parser.add_argument('--odometry-waiting-time', type=float, default=0.1)
    parser.add_argument('--keep-running', action='store_true')
    parser.add_argument('-out-topic', '--out-topic', required=True, type=str)
    return parser


def accumulate_point_clouds(new_point_cloud_msg):
    global odometry_frame_id
    global odometry_child_frame_id
    global number_of_point_clouds_to_accumulate
    global odometry_waiting_time
    global use_odometry_from_tf
    global accumulated_point_cloud_publisher
    global tf_buffer
    global local_tf_buffer
    global keep_running

    accumulate_point_clouds_time = time.time()

    # We work with np.float64 because np.float32 is not precise enough if odometry is too far away, so accumulated point cloud is trembling,
    # but rviz renders only np.float32, so we do all computations in np.float64 and convert final result to np.float32.
    new_point_cloud = pointcloud2_to_xyz_array(new_point_cloud_msg).transpose()
    new_point_cloud = np.vstack((new_point_cloud, np.ones((1, new_point_cloud.shape[1]))))

    lookup_transform_time = time.time()
    if odometry_frame_id is None:
        rospy.logwarn("No odometry messages yet")
        return
    if use_odometry_from_tf:
        try:
            new_point_cloud_pose_tf = tf_buffer.lookup_transform(odometry_frame_id, new_point_cloud_msg.header.frame_id,
                    new_point_cloud_msg.header.stamp, timeout=rospy.Duration.from_sec(odometry_waiting_time))
        except Exception as e:
            if accumulate_point_clouds.first:
                rospy.logwarn("Waiting for first transform. Reason: {}".format(str(e)))
                return
            if keep_running:
                raise
            else:
                rospy.logerr(str(e))
                rospy.signal_shutdown("Error while trying to find transform")
        new_point_cloud_pose = transform_to_numpy(new_point_cloud_pose_tf.transform)
    else:
        try:
            odometry_tf = local_tf_buffer.lookup_transform(odometry_frame_id, odometry_child_frame_id,
                    new_point_cloud_msg.header.stamp, timeout=rospy.Duration.from_sec(odometry_waiting_time))
            sensor_tf = tf_buffer.lookup_transform(odometry_child_frame_id, new_point_cloud_msg.header.frame_id, new_point_cloud_msg.header.stamp)
        except Exception as e:
            if accumulate_point_clouds.first:
                rospy.logwarn("Waiting for first transform. Reason: {}".format(str(e)))
                return
            if keep_running:
                raise
            else:
                rospy.logerr(str(e))
                rospy.signal_shutdown("Error while trying to find transform")
        odometry = transform_to_numpy(odometry_tf.transform)
        sensor = transform_to_numpy(sensor_tf.transform)
        new_point_cloud_pose = np.matmul(odometry, sensor)
    accumulate_point_clouds.first = False
    lookup_transform_time = time.time() - lookup_transform_time
    
    accumulate_point_clouds.point_clouds_with_poses.append([new_point_cloud, new_point_cloud_pose])
    if len(accumulate_point_clouds.point_clouds_with_poses) < number_of_point_clouds_to_accumulate:
        return

    accumulated_point_cloud = np.empty(shape=0, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
    last_pose_inv = np.linalg.inv(new_point_cloud_pose)
    for point_cloud, pose in accumulate_point_clouds.point_clouds_with_poses:
        transformed_point_cloud = np.matmul(np.matmul(last_pose_inv, pose), point_cloud).astype(np.float32)
        accumulated_point_cloud = np.hstack((accumulated_point_cloud, transformed_point_cloud[:3, :].transpose().ravel().view(accumulated_point_cloud.dtype)))

    accumulated_point_cloud_msg = array_to_pointcloud2(accumulated_point_cloud, stamp=new_point_cloud_msg.header.stamp, frame_id=new_point_cloud_msg.header.frame_id)
    accumulated_point_cloud_publisher.publish(accumulated_point_cloud_msg)
    accumulate_point_clouds.point_clouds_with_poses.popleft()

    accumulate_point_clouds_time = time.time() - accumulate_point_clouds_time
    print("Looking up for transform took {} ms".format(lookup_transform_time * 1000))
    print("Processing took {} ms".format((accumulate_point_clouds_time - lookup_transform_time) * 1000))
    print()


def odometry_received(odom):
    global odometry_frame_id
    global odometry_child_frame_id
    global local_tf_buffer

    if not odometry_frame_id:
        odometry_frame_id = odom.header.frame_id
        odometry_child_frame_id = odom.child_frame_id

    transform = TransformStamped()
    transform.header = odom.header
    transform.child_frame_id = odom.child_frame_id
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.translation.z = odom.pose.pose.position.z
    transform.transform.rotation = odom.pose.pose.orientation

    local_tf_buffer.set_transform(odom, 'default_authority')
    

def transform_received(transform):
    global odometry_frame_id
    global odometry_child_frame_id
    global local_tf_buffer

    if not odometry_frame_id:
        odometry_frame_id = transform.header.frame_id
        odometry_child_frame_id = transform.child_frame_id

    print("Transform delay: {} ms".format((rospy.Time.now() - transform.header.stamp).to_sec() * 1000))

    local_tf_buffer.set_transform(transform, 'default_authority')


if __name__ == '__main__':
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    odometry_frame_id = args.odometry_frame_id
    number_of_point_clouds_to_accumulate = args.number_of_point_clouds_to_accumulate
    odometry_waiting_time = args.odometry_waiting_time
    use_odometry_from_tf = args.odometry_frame_id is not None
    keep_running = args.keep_running

    rospy.init_node('accumulate_point_clouds')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    accumulated_point_cloud_publisher = rospy.Publisher(args.out_topic, PointCloud2, queue_size=10)
    rospy.Subscriber(args.point_cloud_topic, PointCloud2, accumulate_point_clouds)
    if not use_odometry_from_tf:
        local_tf_buffer  = tf2_ros.Buffer(debug=False)
        if args.odometry_topic:
            rospy.Subscriber(args.odometry_topic, Odometry, odometry_received)
        elif args.transforms_topic:
            rospy.Subscriber(args.transforms_topic, TransformStamped, transform_received)
        odometry_child_frame_id = None
    accumulate_point_clouds.point_clouds_with_poses = deque()
    accumulate_point_clouds.first = True
    rospy.spin()
