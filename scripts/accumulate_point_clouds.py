#!/usr/bin/env python3
import rospy
import tf2_ros
import argparse
import numpy as np
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array, array_to_pointcloud2
from ros_numpy.geometry import transform_to_numpy
from sensor_msgs.msg import PointCloud2
from collections import deque
import time


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--point-cloud-topic', required=True, type=str)
    parser.add_argument('--odometry-frame-id', required=True, type=str)
    parser.add_argument('-num', '--number-of-point-clouds-to-accumulate', required=True, type=int)
    parser.add_argument('-out-topic', '--out-topic', required=True, type=str)
    return parser


def accumulate_point_clouds(new_point_cloud_msg: PointCloud2):
    global odometry_frame_id
    global number_of_point_clouds_to_accumulate
    global accumulated_point_cloud_publisher
    global tf_buffer

    accumulate_point_clouds_time = time.time()

    # We work with np.float64 because np.float32 is not precise enough and accumulated point cloud is trembling,
    # but rviz renders only np.float32, so we do all computations in np.float64 and convert final result to np.float32.
    new_point_cloud = pointcloud2_to_xyz_array(new_point_cloud_msg).transpose()
    new_point_cloud = np.vstack((new_point_cloud, np.ones((1, new_point_cloud.shape[1]))))
    trying_to_lookup_transform_time = time.time()
    while True:
        try:
            lookup_transform_time = time.time()
            new_point_cloud_pose_tf = tf_buffer.lookup_transform(odometry_frame_id, new_point_cloud_msg.header.frame_id, new_point_cloud_msg.header.stamp)
            lookup_transform_time = time.time() - lookup_transform_time
            break
        except tf2_ros.ExtrapolationException:
            if time.time() - trying_to_lookup_transform_time > 0.1:
                raise
            rospy.sleep(0.01)
    trying_to_lookup_transform_time = time.time() - trying_to_lookup_transform_time
    new_point_cloud_pose = transform_to_numpy(new_point_cloud_pose_tf.transform)
    accumulate_point_clouds.point_clouds_with_poses.append([new_point_cloud, new_point_cloud_pose])

    if len(accumulate_point_clouds.point_clouds_with_poses) < number_of_point_clouds_to_accumulate:
        return

    accumulated_point_cloud = np.array(0, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
    last_pose_inv = np.linalg.inv(new_point_cloud_pose)
    for (point_cloud, pose) in accumulate_point_clouds.point_clouds_with_poses:
        transformed_point_cloud = np.matmul(np.matmul(last_pose_inv, pose), point_cloud).astype(np.float32)
        accumulated_point_cloud = np.hstack((accumulated_point_cloud, transformed_point_cloud[:3, :].transpose().ravel().view(accumulated_point_cloud.dtype)))

    accumulated_point_cloud_msg = array_to_pointcloud2(accumulated_point_cloud, stamp=new_point_cloud_msg.header.stamp, frame_id=new_point_cloud_msg.header.frame_id)
    accumulated_point_cloud_publisher.publish(accumulated_point_cloud_msg)
    accumulate_point_clouds.point_clouds_with_poses.popleft()

    accumulate_point_clouds_time = time.time() - accumulate_point_clouds_time
    print("Waiting for transform took {} ms".format((trying_to_lookup_transform_time - lookup_transform_time) * 1000))
    print("Processing took {} ms".format((accumulate_point_clouds_time - trying_to_lookup_transform_time + lookup_transform_time) * 1000))
    print()


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    odometry_frame_id = args.odometry_frame_id
    number_of_point_clouds_to_accumulate = args.number_of_point_clouds_to_accumulate

    rospy.init_node('accumulate_point_clouds')
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Time(2))
    listener = tf2_ros.TransformListener(tf_buffer)
    accumulated_point_cloud_publisher = rospy.Publisher(args.out_topic, PointCloud2, queue_size=1)
    rospy.Subscriber(args.point_cloud_topic, PointCloud2, accumulate_point_clouds)
    accumulate_point_clouds.point_clouds_with_poses = deque()
    rospy.spin()
