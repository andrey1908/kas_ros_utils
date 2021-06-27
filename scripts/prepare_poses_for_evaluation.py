#!/usr/bin/env python3
from math import e
import rosbag
import rospy
import roslib
from urdf_parser_py.urdf import URDF
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Transform
import tf2_ros
from matplotlib import pyplot as plt
from tqdm import tqdm
import argparse
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2quat
import numpy as np


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-bag-gt', '--rosbag-gt-file', required=True, type=str, help=".bag file with gt poses")
    parser.add_argument('-gt-topic', '--gt-topic', required=True, type=str, help="topic to read gt poses")

    parser.add_argument('-bag-res', '--rosbag-results-file', required=True, type=str, help=".bag file with SLAM trajectory")
    parser.add_argument('-res-topic', '--results-topic', required=True, type=str, help="topic to read SLAM trajectory")

    parser.add_argument('-static-transforms', '--static-transforms-file', type=str)

    parser.add_argument('-out-gt', '--out-gt-file', required=True, type=str, help="output file with gt poses in kitti format")
    parser.add_argument('-out-res', '--out-results-file', required=True, type=str, help="output file with SLAM poses in kitti format")

    parser.add_argument('-out-paths', '--out-paths-file', type=str)
    return parser


def is_ascending(list):
    previous = list[0]
    for number in list:
        if previous > number:
            return False
        previous = number
    return True


def find_mutual_indexes(A, B, max_error=0.01):
    if not is_ascending(A):
        raise(RuntimeError)
    if not is_ascending(B):
        raise(RuntimeError)

    swapped = False
    if len(A) > len(B):
        A, B = B, A
        swapped = True

    begin = max(A[0], B[0])
    end = min(A[-1], B[-1])
    
    A_indexes = list()
    B_indexes = list()
    discarded_due_to_large_error = 0
    for A_index, a in enumerate(A):
        B_index = np.argmin(np.abs(B - a))
        b = B[B_index]
        if (a < begin) or (b < begin):
            continue
        if (a > end) or (b > end):
            break
        if abs(a - b) > max_error:
            discarded_due_to_large_error += 1
            continue
        A_indexes.append(A_index)
        B_indexes.append(B_index)

    print('Number of discarded indexes due to large error: {}'.format(discarded_due_to_large_error))
    if swapped:
        A, B = B, A
        A_indexes, B_indexes = B_indexes, A_indexes
    print('Found {} mutual indexes in arrays with {} and {} elements'.format(len(A_indexes), len(A), len(B)))
    return A_indexes, B_indexes


def to_matrix(Q, T):
    M = np.zeros((4, 4))
    M[:3, :3] = quat2mat([Q.w, Q.x, Q.y, Q.z])
    M[:3, 3] = [T.x, T.y, T.z]
    M[3, 3] = 1
    return M


def ros_msg_to_matrix(ros_msg):
    if hasattr(ros_msg, 'pose'):
        return to_matrix(ros_msg.pose.pose.orientation, ros_msg.pose.pose.position)
    if hasattr(ros_msg, 'transform'):
        return to_matrix(ros_msg.transform.rotation, ros_msg.transform.translation)
    raise(TypeError("Unknown type {}".format(type(ros_msg))))


def matrix_to_ros_pose(M):
    ros_pose = Pose()
    ros_pose.position.x, ros_pose.position.y, ros_pose.position.z = M[:3, 3]
    ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z = mat2quat(M[:3, :3])
    return ros_pose


def fill_tf_buffer_with_static_transforms_from_bag(bag, tfBuffer):
    for topic, msg, t in bag.read_messages(topics=['/tf_static']):
        for transform in msg.transforms:
            tfBuffer.set_transform_static(transform, 'default_authority')
            

def fill_tf_buffer_with_static_transforms_from_urdf(urdf, tfBuffer):
    for joint in urdf.joints:
        if joint.type != 'fixed':
            continue
        ros_transform = Transform()
        ros_transform.translation.x, ros_transform.translation.y, ros_transform.translation.z = joint.origin.xyz
        ros_transform.rotation.w, ros_transform.rotation.x, ros_transform.rotation.y, ros_transform.rotation.z = euler2quat(*joint.origin.rpy, axes='sxyz')
        transform_stamped = TransformStamped(transform=ros_transform)
        transform_stamped.header.frame_id = joint.parent
        transform_stamped.header.stamp = rospy.Time()
        transform_stamped.child_frame_id = joint.child
        tfBuffer.set_transform_static(transform_stamped, 'default_authority')


def fill_tf_buffer_with_static_transforms(transforms_source, transforms_source_format, tfBuffer):
    if transforms_source_format == '.bag':
        fill_tf_buffer_with_static_transforms_from_bag(transforms_source, tfBuffer)
    elif transforms_source_format == '.urdf':
        fill_tf_buffer_with_static_transforms_from_urdf(transforms_source, tfBuffer)


def read_poses(bag, topic):
    timestamps = list()
    poses = list()
    child_frame_id = None
    for tpc, msg, t in tqdm(bag.read_messages(topics=[topic]), total=bag.get_message_count(topic_filters=[topic])):
        if child_frame_id:
            if child_frame_id != msg.child_frame_id:
                raise(RuntimeError)
        else:
            child_frame_id = msg.child_frame_id
        timestamps.append(msg.header.stamp.to_sec())
        poses.append(ros_msg_to_matrix(msg))
    return timestamps, poses, child_frame_id


def print_info(gt_timestamps, gt_indexes, results_timestamps, results_indexes):
    gt_indexed_timestamps = gt_timestamps[gt_indexes]
    results_indexed_timestamps = results_timestamps[results_indexes]

    max_error = np.max(np.abs(gt_indexed_timestamps - results_indexed_timestamps))
    print("Max error: {} ms".format(max_error * 1000))

    gt_indexed_steps = np.abs(np.insert(gt_indexed_timestamps, 0, gt_indexed_timestamps[0]) - \
        np.append(gt_indexed_timestamps, gt_indexed_timestamps[-1]))[1:-1]
    results_indexed_steps = np.abs(np.insert(results_indexed_timestamps, 0, results_indexed_timestamps[0]) - \
        np.append(results_indexed_timestamps, results_indexed_timestamps[-1]))[1:-1]
    max_indexed_step = max(np.max(gt_indexed_steps), np.max(results_indexed_steps))
    print('Max step in indexed timestamps: {} ms'.format(max_indexed_step * 1000))

    gt_steps = np.abs(np.insert(gt_timestamps, 0, gt_timestamps[0]) - \
        np.append(gt_timestamps, gt_timestamps[-1]))[1:-1]
    results_steps = np.abs(np.insert(results_timestamps, 0, results_timestamps[0]) - \
        np.append(results_timestamps, results_timestamps[-1]))[1:-1]
    plt.plot(gt_steps)
    #plt.show()
    plt.plot(results_steps)
    #plt.show()
    

def dump_poses(out_file, poses, transform=np.eye(4)):
    with open(out_file, 'w') as f:
        origin_pose_inv = None
        for i, pose in enumerate(poses):
            # remember the first pose
            if origin_pose_inv is None:
                origin_pose_inv = np.linalg.inv(pose)
            # move odometry to the origin
            pose = origin_pose_inv @ pose
            # convert poses to another coordinate system
            pose = transform @ pose @ np.linalg.inv(transform)
            out_pose_str = '{:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e}'.format(
                    pose[0][0], pose[0][1], pose[0][2], pose[0][3],
                    pose[1][0], pose[1][1], pose[1][2], pose[1][3],
                    pose[2][0], pose[2][1], pose[2][2], pose[2][3])
            if i != len(poses) - 1:
                out_pose_str = out_pose_str + '\n'
            f.write(out_pose_str)


def dump_path(out_bag, topic, poses, timestamps, transform=np.eye(4)):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.from_sec(timestamps[0])
    origin_pose_inv = None
    for pose, timestamp in zip(poses, timestamps):
        # remember the first pose
        if origin_pose_inv is None:
            origin_pose_inv = np.linalg.inv(pose)
        # move odometry to the origin
        pose = origin_pose_inv @ pose
        # convert poses to another coordinate system
        pose = transform @ pose @ np.linalg.inv(transform)
        ros_pose = matrix_to_ros_pose(pose)
        pose_stamped = PoseStamped(pose=ros_pose)
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = rospy.Time.from_sec(timestamp)
        path.poses.append(pose_stamped)
    out_bag.write(topic, path, path.header.stamp)


def prepare_poses_for_evaluation(rosbag_gt_file, gt_topic, rosbag_results_file, results_topic,
                                 static_transforms_file, out_gt_file, out_results_file, out_paths_file=None):
    print("Loading files...")
    bag_gt = rosbag.Bag(rosbag_gt_file)
    bag_results = rosbag.Bag(rosbag_results_file)
    transforms_source = None
    transforms_source_format = str()
    if static_transforms_file:
        if static_transforms_file.endswith('.bag'):
            transforms_source = rosbag.Bag(static_transforms_file)
            transforms_source_format = '.bag'
        elif static_transforms_file.endswith('.urdf'):
            transforms_source = URDF.from_xml_file(static_transforms_file)
            transforms_source_format = '.urdf'
        else:
            raise(RuntimeError)
    print("Files loaded")

    tfBuffer = tf2_ros.Buffer()
    if transforms_source is not None:
        print("Read static transforms")
        fill_tf_buffer_with_static_transforms(transforms_source, transforms_source_format, tfBuffer)

    print("Extracting poses...")
    gt_timestamps, gt_poses, gt_child_frame_id = read_poses(bag_gt, gt_topic)
    results_timestamps, results_poses, results_child_frame_id = read_poses(bag_results, results_topic)
    if not is_ascending(gt_timestamps):
        raise(RuntimeError)
    if not is_ascending(results_timestamps):
        raise(RuntimeError)
    gt_timestamps = np.array(gt_timestamps)
    gt_poses = np.array(gt_poses)
    results_timestamps = np.array(results_timestamps)
    results_poses = np.array(results_poses)
    print("Poses extracted")

    if gt_child_frame_id != results_child_frame_id:
        print("Retrive transform for SLAM poses")
        if transforms_source is None:
            raise(RuntimeError)
        ros_transform = tfBuffer.lookup_transform(gt_child_frame_id, results_child_frame_id, rospy.Time(0))
        transform_for_results = ros_msg_to_matrix(ros_transform)
    else:
        transform_for_results = np.eye(4)

    print("Finding mutual indexes for poses...")
    gt_indexes, results_indexes = find_mutual_indexes(gt_timestamps, results_timestamps)
    if not is_ascending(gt_indexes):
        raise(RuntimeError)
    if not is_ascending(results_indexes):
        raise(RuntimeError)
    print_info(gt_timestamps, gt_indexes, results_timestamps, results_indexes)

    print("Dump poses in kitti format")
    dump_poses(out_gt_file, gt_poses[gt_indexes])
    dump_poses(out_results_file, results_poses[results_indexes], transform_for_results)

    if out_paths_file:
        print("Dump trajectories in rosbag")
        with rosbag.Bag(out_paths_file, 'w') as out_bag:
            dump_path(out_bag, '/gt_path', gt_poses[gt_indexes], gt_timestamps[gt_indexes])
            dump_path(out_bag, '/results_path', results_poses[results_indexes], results_timestamps[results_indexes], transform_for_results)

    bag_gt.close()
    bag_results.close()
    if transforms_source:
        if transforms_source_format == '.bag':
            transforms_source.close()

    print("Finished!")


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    prepare_poses_for_evaluation(**vars(args))
    