#!/usr/bin/env python3
import rospy
import rosbag
import tf2_ros
from geometry_msgs.msg import TransformStamped
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-from', '--from-frame', required=True, type=str)
    parser.add_argument('-to', '--to-frame', required=True, type=str)
    parser.add_argument('-rate', '--rate', type=int, default=100)
    parser.add_argument('-out-bag', '--out-bag-file', required=True, type=str)
    parser.add_argument('-out-topic', '--out-topic-name', type=str, default='/transforms')
    return parser


def read_transforms(from_frame, to_frame, rate, out_bag_file, out_topic_name):
    r = rospy.Rate(rate)
    last_transform = TransformStamped()
    transforms_written = 0
    with rosbag.Bag(out_bag_file, 'w') as bag:
        print("Recording...")
        while not rospy.is_shutdown():
            try:
                transform = tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                try:
                    r.sleep()
                except (rospy.ROSInterruptException):
                    break
                continue
            if last_transform != transform:
                bag.write(out_topic_name, transform, rospy.get_rostime())
                transforms_written += 1
                last_transform = transform
                print("\r{} transforms written".format(transforms_written), end='')
            try:
                r.sleep()
            except (rospy.ROSInterruptException):
                break
    print("\nFinished!")


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    
    rospy.init_node('read_transforms')
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    read_transforms(**vars(args))
