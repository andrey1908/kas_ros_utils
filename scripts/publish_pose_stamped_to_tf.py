#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-pose-stamped', '--pose-stamped-topic', required=True, type=str)
    parser.add_argument('-child', '--child-frame-id', required=True, type=str)
    return parser


def publish_pose_stamped_to_tf(pose_stamped: PoseStamped):
    global tf_broadcaster, child_frame_id

    transform = TransformStamped()
    transform.header = pose_stamped.header
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = pose_stamped.pose.position.x
    transform.transform.translation.y = pose_stamped.pose.position.y
    transform.transform.translation.z = pose_stamped.pose.position.z
    transform.transform.rotation = pose_stamped.pose.orientation

    tf_broadcaster.sendTransform(transform)


if __name__ == '__main__':
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    pose_stamped_topic = args.pose_stamped_topic
    child_frame_id = args.child_frame_id
    
    rospy.init_node('publish_pose_stamped_to_tf')
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber(pose_stamped_topic, PoseStamped, publish_pose_stamped_to_tf)

    print("Spinning...")
    rospy.spin()
    
