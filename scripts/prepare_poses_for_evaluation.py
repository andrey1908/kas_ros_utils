#!/usr/bin/env python
import rosbag
import rospy
import tf2_ros
import argparse
import os
import os.path as osp
from static_transforms_reader import fill_tf_buffer_with_static_transforms_from_file
from poses_handler import read_poses_from_bag_files, get_union_intersection_time_difference, match_poses, get_max_time_step, \
    align_poses, move_first_pose_to_the_origin, transform_poses, write_poses, poses_to_ros_path, is_ascending
from ros_numpy.geometry import transform_to_numpy


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-gt-bags', '--gt-rosbag-files', required=True, type=str, nargs='+', help=".bag files with gt poses")
    parser.add_argument('-gt-topic', '--gt-topic', required=True, type=str, help="topic to read gt poses")

    parser.add_argument('-res-bags', '--results-rosbag-files', required=True, type=str, nargs='+', help=".bag files with SLAM poses")
    parser.add_argument('-res-topic', '--results-topic', required=True, type=str, help="topic to read SLAM poses")

    parser.add_argument('-out-gt', '--out-gt-file', required=True, type=str, help="output file with gt poses in kitti format")
    parser.add_argument('-out-res', '--out-results-file', required=True, type=str, help="output file with SLAM poses in kitti format")

    parser.add_argument('-transforms-source', '--transforms-source-file', type=str, help=".bag, .urdf or .launch file to read static transforms if needed")
    parser.add_argument('-out-trajectories', '--out-trajectories-rosbag-file', type=str, help="output .bag file to write gt and SLAM trajectories")

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.5,
        help="Max difference between union and intersection of time ragnes where gt and SLAM poses are set.")
    parser.add_argument('--max-time-error', type=float, default=0.01, help="Max time error during matching gt and SLAM poses.")
    parser.add_argument('--max-time-step', type=float, default=0.23, help="Max time step in gt and SLAM poses after matching.")
    return parser


def prepare_poses_for_evaluation(gt_rosbag_files, gt_topic, results_rosbag_files, results_topic,
                                 out_gt_file, out_results_file, transforms_source_file=None, out_trajectories_rosbag_file=None,
                                 max_union_intersection_time_difference=0.5, max_time_error=0.01, max_time_step=0.23):
    if not osp.exists(osp.dirname(osp.realpath(out_gt_file))):
        os.makedirs(osp.dirname(osp.realpath(out_gt_file)))  # osp.realpath is needed because if out_gt_file is a file name in current directory,
                                                             # then osp.dirname will be '' and os.makedirs fails
    if not osp.exists(osp.dirname(osp.realpath(out_results_file))):
        os.makedirs(osp.dirname(osp.realpath(out_results_file)))  # same here

    print("Extracting poses...")
    if isinstance(gt_rosbag_files, str):
        gt_rosbag_files = [gt_rosbag_files]
    if isinstance(results_rosbag_files, str):
        results_rosbag_files = [results_rosbag_files]
    gt_poses, gt_timestamps, _, gt_child_frame_id = read_poses_from_bag_files(gt_rosbag_files, gt_topic, use_tqdm=True)
    results_poses, results_timestamps, _, results_child_frame_id = read_poses_from_bag_files(results_rosbag_files, results_topic, use_tqdm=True)
    if not is_ascending(gt_timestamps):
        raise RuntimeError("Gt poses not sorted")
    if not is_ascending(results_timestamps):
        raise RuntimeError("Results poses not sorted")

    union_intersection_time_difference = get_union_intersection_time_difference(gt_timestamps, results_timestamps)
    print("Union intersection difference: {:.3f} s".format(union_intersection_time_difference))
    if union_intersection_time_difference > max_union_intersection_time_difference:
        raise RuntimeError("Union intersection difference is {:.3f}, but it should not be greater than {:.3f}".format(
            union_intersection_time_difference, max_union_intersection_time_difference))

    print("Matching poses...")
    matched_gt_poses, matched_gt_timestamps, matched_results_poses, matched_results_timestamps, max_matching_error = \
        match_poses(gt_poses, gt_timestamps, results_poses, results_timestamps, max_time_error=max_time_error)
    print('Found {} mutual indexes in arrays with {} and {} elements'.format(len(matched_gt_poses), len(gt_poses), len(results_poses)))
    print('Max error: {:.3f} ms'.format(max_matching_error * 1000))

    max_step = max(get_max_time_step(matched_gt_timestamps), get_max_time_step(matched_results_timestamps))
    print('Max step in matched timestamps: {:.3f} s'.format(max_step))
    if max_step > max_time_step:
        raise RuntimeError("Max step in matched poses is {:.3f}, but it should not be greater than {:.3f}".format(max_step, max_time_step))

    print("Aligning poses...")
    aligned_gt_poses, aligned_results_poses, aligned_timestamps = \
        align_poses(matched_gt_poses, matched_gt_timestamps, matched_results_poses, matched_results_timestamps)

    print("Moving poses to the origin...")
    move_first_pose_to_the_origin(aligned_gt_poses)
    move_first_pose_to_the_origin(aligned_results_poses)

    if gt_child_frame_id != results_child_frame_id:
        if not transforms_source_file:
            raise RuntimeError("Need transforms source file to convert poses between "
                "gt frame '{}' and results frame '{}'".format(gt_child_frame_id, results_child_frame_id))
        print("Reading static transforms...")
        tf_buffer = tf2_ros.Buffer(debug=False)
        fill_tf_buffer_with_static_transforms_from_file(transforms_source_file, tf_buffer)
        ros_transform = tf_buffer.lookup_transform(results_child_frame_id, gt_child_frame_id, rospy.Time())
        transform = transform_to_numpy(ros_transform.transform)
        print("Transforming SLAM poses to gt frame...")
        transform_poses(aligned_results_poses, transform)

    print("Writing poses in kitti format...")
    write_poses(out_gt_file, aligned_gt_poses)
    write_poses(out_results_file, aligned_results_poses)

    if out_trajectories_rosbag_file:
        print("Writing trajectories in rosbag...")
        gt_path = poses_to_ros_path(aligned_gt_poses, aligned_timestamps)
        results_path = poses_to_ros_path(aligned_results_poses, aligned_timestamps)
        with rosbag.Bag(out_trajectories_rosbag_file, 'w') as out_bag:
            out_bag.write('/gt_path', gt_path, gt_path.header.stamp)
            out_bag.write('/results_path', results_path, results_path.header.stamp)

    print("Finished!")


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    prepare_poses_for_evaluation(**vars(args))
