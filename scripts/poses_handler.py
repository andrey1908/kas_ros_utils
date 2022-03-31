from tqdm import tqdm
import numpy as np
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from nav_msgs.msg import Path
from ros_numpy.geometry import transform_to_numpy, pose_to_numpy, numpy_to_pose
from tf.transformations import quaternion_from_matrix, quaternion_slerp, quaternion_matrix


def is_ascending(list):
    previous = list[0]
    for number in list:
        if previous > number:
            return False
        previous = number
    return True


def ros_to_numpy(ros_message):
    if ros_message._type == 'geometry_msgs/TransformStamped':
        # workaround
        ros_message.transform.translation.__class__ = Vector3
        ros_message.transform.rotation.__class__ = Quaternion
        return transform_to_numpy(ros_message.transform)
    if ros_message._type == 'nav_msgs/Odometry':
        # workaround
        ros_message.pose.pose.position.__class__ = Point
        ros_message.pose.pose.orientation.__class__ = Quaternion
        return pose_to_numpy(ros_message.pose.pose)
    raise TypeError("Unknown type {}".format(type(ros_message)))


def read_poses(bag, topic, use_tqdm=False):
    poses = list()
    timestamps = list()
    frame_id = None
    child_frame_id = None
    if use_tqdm:
        bag_reader = tqdm(bag.read_messages(topics=[topic]), total=bag.get_message_count(topic_filters=[topic]))
    else:
        bag_reader = bag.read_messages(topics=[topic])
    for _, msg, _ in bag_reader:
        if child_frame_id:
            if child_frame_id != msg.child_frame_id:
                raise RuntimeError
        else:
            child_frame_id = msg.child_frame_id
        if frame_id:
            if frame_id != msg.header.frame_id:
                raise RuntimeError
        else:
            frame_id = msg.header.frame_id
        poses.append(ros_to_numpy(msg))
        timestamps.append(msg.header.stamp.to_sec())
    return poses, timestamps, frame_id, child_frame_id


def read_poses_from_bag_files(rosbag_files, topic, use_tqdm=False):
    if isinstance(rosbag_files, str):
        rosbag_files = [rosbag_files]
    combined_poses = list()
    combined_timestamps = list()
    prev_frame_id = None
    prev_child_frame_id = None
    for rosbag_file in rosbag_files:
        with rosbag.Bag(rosbag_file) as bag:
            poses, timestamps, frame_id, child_frame_id = read_poses(bag, topic, use_tqdm=use_tqdm)
            combined_poses += poses
            combined_timestamps += timestamps
            if prev_frame_id is not None:
                if (prev_frame_id != frame_id) or (prev_child_frame_id != child_frame_id):
                    raise RuntimeError()
            prev_frame_id = frame_id
            prev_child_frame_id = child_frame_id
    return combined_poses, combined_timestamps, frame_id, child_frame_id


def get_union_intersection_time_difference(timestamps1, timestamps2):
    sorted_timestamps_1 = sorted(timestamps1)
    sorted_timestamps_2 = sorted(timestamps2)

    union = max(sorted_timestamps_1[-1], sorted_timestamps_2[-1]) - min(sorted_timestamps_1[0], sorted_timestamps_2[0])
    intersection = min(sorted_timestamps_1[-1], sorted_timestamps_2[-1]) - max(sorted_timestamps_1[0], sorted_timestamps_2[0])
    if intersection < 0:
        return union + intersection
    else:
        return union - intersection


def get_max_time_step(timestamps):
    if not is_ascending(timestamps):
        raise ValueError("get_max_time_step() got unsorted timestamps")

    max_time_step = 0
    previous_timestamp = timestamps[0]
    for timestamp in timestamps:
        time_step = timestamp - previous_timestamp
        max_time_step = max(max_time_step, time_step)
        previous_timestamp = timestamp
    return max_time_step


def move_first_pose_to_the_origin(poses):
    first_pose_inv = np.linalg.inv(poses[0])
    for i in range(len(poses)):
        poses[i] = np.matmul(first_pose_inv, poses[i])


# 'transform' moves frame that is used now to desired frame
def transform_poses(poses, transform):
    transform_inv = np.linalg.inv(transform)
    for i in range(len(poses)):
        poses[i] = np.matmul(np.matmul(transform_inv, poses[i]), transform)


def find_boundary_indexes(array, value):
    if len(array) == 0:
        raise RuntimeError("Should not happen")
    lower = None
    lower_min_difference = -1
    upper = None
    upper_min_difference = -1
    for i in range(len(array)):
        if array[i] == value:
            raise RuntimeError("Should not happen")
        if array[i] < value:
            if lower_min_difference > value - array[i] or lower_min_difference < 0:
                lower = i
                lower_min_difference = value - array[i]
        if array[i] > value:
            if upper_min_difference > array[i] - value or upper_min_difference < 0:
                upper = i
                upper_min_difference = array[i] - value
    return lower, upper


def match_poses(poses1, timestamps1, poses2, timestamps2, max_time_error=0.01):
    if not is_ascending(timestamps1):
        raise ValueError("match_poses() got unsorted timestamps1")
    if not is_ascending(timestamps2):
        raise ValueError("match_poses() got unsorted timestamps2")
    if len(poses1) != len(timestamps1):
        raise ValueError("Different number of poses1 ({}) and timestamps1 ({})".format(len(poses1), len(timestamps1)))
    if len(poses2) != len(timestamps2):
        raise ValueError("Different number of poses2 ({}) and timestamps2 ({})".format(len(poses2), len(timestamps2)))

    matching_results = list()
    start_index_2 = 0
    for index_1 in range(len(timestamps1)):
        while timestamps1[index_1] - timestamps2[start_index_2] > max_time_error:
            start_index_2 += 1
            if start_index_2 == len(timestamps2):
                break
        if start_index_2 == len(timestamps2):
            break
        index_2 = start_index_2
        while timestamps2[index_2] - timestamps1[index_1] <= max_time_error:
            matching_results.append((abs(timestamps1[index_1] - timestamps2[index_2]), index_1, index_2))
            index_2 += 1
            if index_2 == len(timestamps2):
                break

    if len(matching_results) == 0:
        raise RuntimeError("match_poses() cound not find any matches. Try to increase max_time_error for matching.")
    matching_results.sort()

    matched_indexes_1 = set()
    matched_indexes_2 = set()
    indexes_1 = list()
    indexes_2 = list()
    max_matching_error = 0
    for matching_result in matching_results:
        index_1 = matching_result[1]
        index_2 = matching_result[2]
        if (index_1 in matched_indexes_1) or (index_2 in matched_indexes_2):
            continue
        if len(indexes_1) > 0:
            lower, upper = find_boundary_indexes(indexes_1, index_1)
            lower_B_condition = True
            upper_B_condition = True
            if lower is not None:
                lower_B_condition = indexes_2[lower] < index_2
            if upper is not None:
                upper_B_condition = indexes_2[upper] > index_2
            if not lower_B_condition or not upper_B_condition:
                continue
        matched_indexes_1.add(index_1)
        matched_indexes_2.add(index_2)
        indexes_1.append(index_1)
        indexes_2.append(index_2)
        max_matching_error = max(max_matching_error, matching_result[0])

    indexes_1, indexes_2 = map(list, zip(*sorted(zip(indexes_1, indexes_2))))
    if not is_ascending(indexes_1) or not is_ascending(indexes_2):
        raise RuntimeError("Matched indexes are not sorted after matching with match_poses(). This should not happen.")

    matched_poses_1 = list()
    matched_timestamps_1 = list()
    matched_poses_2 = list()
    matched_timestamps_2 = list()
    for index_1, index_2 in zip(indexes_1, indexes_2):
        matched_poses_1.append(poses1[index_1])
        matched_timestamps_1.append(timestamps1[index_1])
        matched_poses_2.append(poses2[index_2])
        matched_timestamps_2.append(timestamps2[index_2])

    return matched_poses_1, matched_timestamps_1, matched_poses_2, matched_timestamps_2, max_matching_error


def interpolate_pose(pose1, timestamp1, pose2, timestamp2, interpolated_timestamp):
    if not timestamp1 <= interpolated_timestamp <= timestamp2:
        raise ValueError("Condition timestamp1 <= interpolated_timestamp <= timestamp2 is not met \
({} <= {} <= {} is not true)".format(timestamp1, interpolated_timestamp, timestamp2))

    if timestamp1 == timestamp2:
        if not (pose1 == pose2).all():
            raise RuntimeError("Can not interpolate: timestamp1 == timestamp2, but pose1 != pose2")
        return pose1.copy()

    fraction = (interpolated_timestamp - timestamp1) / (timestamp2 - timestamp1)

    quaternion1 = quaternion_from_matrix(pose1)
    quaternion2 = quaternion_from_matrix(pose2)
    interpolated_quaternion = quaternion_slerp(quaternion1, quaternion2, fraction)

    translation1 = pose1[:3, 3]
    translation2 = pose2[:3, 3]
    interpolated_translation = (1 - fraction) * translation1 + fraction * translation2

    interpolated_pose = quaternion_matrix(interpolated_quaternion)
    interpolated_pose[:3, 3] = interpolated_translation
    return interpolated_pose


def interpolate_poses(poses, timestamps, interpolated_timestamps):
    if len(poses) != len(timestamps):
        raise ValueError("Different number of poses ({}) and timestamps ({})".format(len(poses), len(timestamps)))
    if not is_ascending(interpolated_timestamps):
        raise ValueError("interpolate_poses() got unsorted interpolated_timestamps")

    workaround_list = list(range(len(poses)))
    # can't compare numpy arrays, so use workaround_list to avoid it
    sorted_timestamps, _, sorted_poses = map(list, zip(*sorted(zip(timestamps, workaround_list, poses))))

    if interpolated_timestamps[0] < sorted_timestamps[0]:
        raise RuntimeError("Timestamp for interpolation {} is less than \
the earliest pose timestamp {}".format(interpolated_timestamps[0], sorted_timestamps[0]))
    index = 0
    interpolated_poses = list()
    for interpolated_timestamp in interpolated_timestamps:
        while sorted_timestamps[index] < interpolated_timestamp:
            index += 1
            if index == len(sorted_timestamps):
                raise RuntimeError("Timestamp for interpolation {} is greater than \
the latest pose timestamp {}".format(interpolated_timestamp, sorted_timestamps[-1]))

        if interpolated_timestamp == sorted_timestamps[index]:
            interpolated_poses.append(sorted_poses[index])
            continue

        interpolated_poses.append(interpolate_pose(
            sorted_poses[index-1], sorted_timestamps[index-1], sorted_poses[index], sorted_timestamps[index], interpolated_timestamp))

    return interpolated_poses


def align_poses(poses1, timestamps1, poses2, timestamps2):
    if not is_ascending(timestamps1):
        raise ValueError("align_poses() got unsorted timestamps1")
    if not is_ascending(timestamps2):
        raise ValueError("align_poses() got unsorted timestamps2")
    if len(poses1) != len(timestamps1) or len(poses1) != len(poses2) or len(poses1) != len(timestamps2):
        raise ValueError("Different number of poses1 ({}), timestamps1 ({}), poses2 ({}) or timestamps2 ({})".format(\
            len(poses1), len(timestamps1), len(poses2), len(timestamps2)))

    min_interpolation_timestamp = max(timestamps1[0], timestamps2[0])
    max_interpolation_timestamp = min(timestamps1[-1], timestamps2[-1])

    alinged_timestamps = list()
    for timestamp1, timestamp2 in zip(timestamps1, timestamps2):
        alinged_timestamp = (timestamp1 + timestamp2) / 2
        alinged_timestamp = max(min_interpolation_timestamp, alinged_timestamp)
        alinged_timestamp = min(max_interpolation_timestamp, alinged_timestamp)
        alinged_timestamps.append(alinged_timestamp)

    alinged_poses_1 = interpolate_poses(poses1, timestamps1, alinged_timestamps)
    alinged_poses_2 = interpolate_poses(poses2, timestamps2, alinged_timestamps)
    return alinged_poses_1, alinged_poses_2, alinged_timestamps


def write_poses(out_file, poses):
    with open(out_file, 'w') as f:
        for i, pose in enumerate(poses):
            out_pose_str = '{:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e}'.format(
                    pose[0][0], pose[0][1], pose[0][2], pose[0][3],
                    pose[1][0], pose[1][1], pose[1][2], pose[1][3],
                    pose[2][0], pose[2][1], pose[2][2], pose[2][3])
            if i != len(poses) - 1:
                out_pose_str = out_pose_str + '\n'
            f.write(out_pose_str)


def poses_to_ros_path(poses, timestamps):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.from_seconds(timestamps[0])
    for pose, timestamp in zip(poses, timestamps):
        ros_pose = numpy_to_pose(pose)
        pose_stamped = PoseStamped(pose=ros_pose)
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = rospy.Time.from_seconds(timestamp)
        path.poses.append(pose_stamped)
    return path
