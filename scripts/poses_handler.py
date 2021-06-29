from tqdm import tqdm
from transforms3d.quaternions import quat2mat, mat2quat
import numpy as np
import rosbag
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import rospy


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


def read_poses(bag, topic, use_tqdm=False):
    timestamps = list()
    poses = list()
    frame_id = None
    child_frame_id = None
    if use_tqdm:
        bag_reader = tqdm(bag.read_messages(topics=[topic]), total=bag.get_message_count(topic_filters=[topic]))
    else:
        bag_reader = bag.read_messages(topics=[topic])
    for tpc, msg, t in bag_reader:
        if child_frame_id:
            if child_frame_id != msg.child_frame_id:
                raise(RuntimeError)
        else:
            child_frame_id = msg.child_frame_id
        if frame_id:
            if frame_id != msg.header.frame_id:
                raise(RuntimeError)
        else:
            frame_id = msg.header.frame_id
        timestamps.append(msg.header.stamp)
        poses.append(ros_msg_to_matrix(msg))
    return timestamps, poses, frame_id, child_frame_id


def read_poses_from_bag_file(rosbag_file, topic, use_tqdm=False):
    with rosbag.Bag(rosbag_file) as bag:
        return read_poses(bag, topic, use_tqdm=use_tqdm)


def move_first_pose_to_the_origin(poses):
    first_pose_inv = np.linalg.inv(poses[0])
    for i in range(len(poses)):
        poses[i] = first_pose_inv @ poses[i]


# 'transform' should move frame that is used now to desired frame
def transform_poses(poses, transform):
    transform_inv = np.linalg.inv(transform)
    for i in range(len(poses)):
        poses[i] = transform_inv @ poses[i] @ transform


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
    path.header.stamp = timestamps[0]
    for pose, timestamp in zip(poses, timestamps):
        ros_pose = matrix_to_ros_pose(pose)
        pose_stamped = PoseStamped(pose=ros_pose)
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = timestamp
        path.poses.append(pose_stamped)
    return path
