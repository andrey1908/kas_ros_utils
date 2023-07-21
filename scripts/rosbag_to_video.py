#!/usr/bin/env python
import argparse
import rosbag
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from tqdm import tqdm


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-rosbag', '--rosbag-file', required=True, type=str)
    parser.add_argument('-t', '--topic', required=True, type=str)
    parser.add_argument('-out', '--out-file', required=True, type=str)
    parser.add_argument('-fourcc', '--fourcc', type=str, default='mp4v')
    parser.add_argument('-fps', '--fps', type=int, default=30)
    return parser


def rosbag_to_video(rosbag_file, topic, out_file, fourcc='mp4v', fps=30):
    bridge = CvBridge()
    with rosbag.Bag(rosbag_file, 'r') as bag:
        out = None
        messages_number = bag.get_message_count(topic)
        if messages_number == 0:
            print('Topic {} not found'.format(topic))
            return
        messages_reader = bag.read_messages(topic)
        for _, msg, t in tqdm(messages_reader, total=messages_number):
            if msg._type == "sensor_msgs/Image":
                image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            elif msg._type == "sensor_msgs/CompressedImage":
                image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            else:
                raise RuntimeError(f"Unknown message type {msg._type}")
            if out is None:
                out = cv2.VideoWriter(out_file, cv2.VideoWriter_fourcc(*fourcc), fps, (image.shape[1], image.shape[0]))
            out.write(image)
        out.release()


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    rosbag_to_video(**vars(args))
