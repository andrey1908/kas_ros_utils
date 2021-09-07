#!/usr/bin/env python3
import rosbag
import rospy
import tf2_ros
import argparse
import numpy as np
from static_transforms_reader import fill_tf_buffer_with_static_transforms_from_file
from poses_handler import read_poses_from_bag_file, move_first_pose_to_the_origin, transform_poses, write_poses, poses_to_ros_path, \
    ros_msg_to_matrix


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-gt-bag', '--gt-rosbag-file', required=True, type=str, help=".bag file with gt poses")
    parser.add_argument('-gt-topic', '--gt-topic', required=True, type=str, help="topic to read gt poses")

    parser.add_argument('-res-bag', '--results-rosbag-file', required=True, type=str, help=".bag file with SLAM trajectory")
    parser.add_argument('-res-topic', '--results-topic', required=True, type=str, help="topic to read SLAM trajectory")

    parser.add_argument('-out-gt', '--out-gt-file', required=True, type=str, help="output file with gt poses in kitti format")
    parser.add_argument('-out-res', '--out-results-file', required=True, type=str, help="output file with SLAM poses in kitti format")

    parser.add_argument('-transforms-source', '--transforms-source-file', type=str, help=".bag or .urdf file to read static transforms from if needed")
    parser.add_argument('-out-trajectories', '--out-trajectories-rosbag-file', type=str, help="output .bag file to write gt and SLAM trajectories")

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.9, help="Max difference between union and intersection or time ragnes where gt and SLAM poses are set.")
    parser.add_argument('--max-time-error', type=float, default=0.01, help="Max time error during matching gt and SLAM poses.")
    parser.add_argument('--max-time-step', type=float, default=0.7, help="Max time step in gt and SLAM poses after matching.")
    return parser


def is_ascending(list):
    previous = list[0]
    for number in list:
        if previous > number:
            return False
        previous = number
    return True


def check_union_intersection_difference(A, B, max_difference=0.9):
    if not is_ascending(A):
        raise(RuntimeError)
    if not is_ascending(B):
        raise(RuntimeError)
    
    union = (max(A[-1], B[-1]) - min(A[0], B[0])).to_sec()
    intersection = (min(A[-1], B[-1]) - max(A[0], B[0])).to_sec()
    print("Union intersection difference: {:.3f} ms".format((union - intersection) * 1000))
    assert(union - intersection <= max_difference)


def find_mutual_indexes(A, B, max_error=0.01):
    # Ascending order is needed for faster filling matching_results variable.
    if not is_ascending(A):
        raise(RuntimeError)
    if not is_ascending(B):
        raise(RuntimeError)

    matching_results = list()
    start_B_index = 0
    for A_index in range(len(A)):
        while (A[A_index] - B[start_B_index]).to_sec() > max_error:
            start_B_index += 1
            if start_B_index == len(B):
                break
        if start_B_index == len(B):
            break
        B_index = start_B_index
        while (B[B_index] - A[A_index]).to_sec() <= max_error:
            matching_results.append((abs((A[A_index] - B[B_index]).to_sec()), A_index, B_index))
            B_index += 1
            if B_index == len(B):
                break

    matching_results.sort()

    matched_A_indexes = set()
    matched_B_indexes = set()
    A_indexes = list()
    B_indexes = list()
    max_matching_error = 0
    for matching_result in matching_results:
        A_index = matching_result[1]
        B_index = matching_result[2]
        if (A_index in matched_A_indexes) or (B_index in matched_B_indexes):
            continue
        matched_A_indexes.add(A_index)
        matched_B_indexes.add(B_index)
        A_indexes.append(A_index)
        B_indexes.append(B_index)
        max_matching_error = max(max_matching_error, matching_result[0])

    A_indexes, B_indexes = map(list, zip(*sorted(zip(A_indexes, B_indexes))))
    
    print('Found {} mutual indexes in arrays with {} and {} elements'.format(len(A_indexes), len(A), len(B)))
    print('Max error: {:.3f} ms'.format(max_matching_error * 1000))
    return A_indexes, B_indexes


def check_step(A, B, max_step=0.7):
    A = np.array(list(map(lambda x: x.to_sec(), A)))
    B = np.array(list(map(lambda x: x.to_sec(), B)))

    A_steps = np.abs(np.insert(A, 0, A[0]) - np.append(A, A[-1]))[1:-1]
    B_steps = np.abs(np.insert(B, 0, B[0]) - np.append(B, B[-1]))[1:-1]
    step = max(np.max(A_steps), np.max(B_steps))
    print('Max step in matched timestamps: {:.3f} ms'.format(step * 1000))
    assert(step <= max_step)


def prepare_poses_for_evaluation(gt_rosbag_file, gt_topic, results_rosbag_file, results_topic,
                                 out_gt_file, out_results_file, transforms_source_file=None, out_trajectories_rosbag_file=None,
                                 max_union_intersection_time_difference=0.9, max_time_error=0.01, max_time_step=0.7):
    print("Extracting poses...")
    gt_timestamps, gt_poses, _, gt_child_frame_id = read_poses_from_bag_file(gt_rosbag_file, gt_topic, use_tqdm=True)
    results_timestamps, results_poses, _, results_child_frame_id = read_poses_from_bag_file(results_rosbag_file, results_topic, use_tqdm=True)
    if not is_ascending(gt_timestamps):
        raise(RuntimeError)
    if not is_ascending(results_timestamps):
        raise(RuntimeError)
    gt_timestamps = np.array(gt_timestamps)
    gt_poses = np.array(gt_poses)
    results_timestamps = np.array(results_timestamps)
    results_poses = np.array(results_poses)

    check_union_intersection_difference(gt_timestamps, results_timestamps, max_difference=max_union_intersection_time_difference)

    print("Finding mutual indexes for poses...")
    gt_indexes, results_indexes = find_mutual_indexes(gt_timestamps, results_timestamps, max_error=max_time_error)
    if not is_ascending(gt_indexes):
        raise(RuntimeError)
    if not is_ascending(results_indexes):
        raise(RuntimeError)

    print("Getting poses with mutual indexes...")
    gt_poses = gt_poses[gt_indexes]
    gt_timestamps = gt_timestamps[gt_indexes]
    results_poses = results_poses[results_indexes]
    results_timestamps = results_timestamps[results_indexes]

    check_step(gt_timestamps, results_timestamps, max_step=max_time_step)

    print("Moving poses to the origin...")
    move_first_pose_to_the_origin(gt_poses)
    move_first_pose_to_the_origin(results_poses)

    if gt_child_frame_id != results_child_frame_id:
        if not transforms_source_file:
            raise(RuntimeError)
        print("Reading static transforms...")
        tf_buffer = tf2_ros.Buffer()
        fill_tf_buffer_with_static_transforms_from_file(transforms_source_file, tf_buffer)
        ros_transform = tf_buffer.lookup_transform(results_child_frame_id, gt_child_frame_id, rospy.Time())
        transform = ros_msg_to_matrix(ros_transform)
        print("Transforming SLAM poses to gt frame...")
        transform_poses(results_poses, transform)

    print("Writing poses in kitti format...")
    write_poses(out_gt_file, gt_poses)
    write_poses(out_results_file, results_poses)

    if out_trajectories_rosbag_file:
        print("Writing trajectories in rosbag...")
        gt_path = poses_to_ros_path(gt_poses, gt_timestamps)
        results_path = poses_to_ros_path(results_poses, results_timestamps)
        with rosbag.Bag(out_trajectories_rosbag_file, 'w') as out_bag:
            out_bag.write('/gt_path', gt_path, gt_path.header.stamp)
            out_bag.write('/results_path', results_path, results_path.header.stamp)

    print("Finished!")


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    prepare_poses_for_evaluation(**vars(args))
    