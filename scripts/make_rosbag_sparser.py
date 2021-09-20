#!/usr/bin/env python3
import rosbag
import rospy
from tqdm import tqdm
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-in', '--in-rosbag-file', required=True, type=str)
    parser.add_argument('-out', '--out-rosbag-file', required=True, type=str)
    parser.add_argument('-ts', '--topics-to-make-sparser', type=str, nargs='+')
    parser.add_argument('-k', type=float)
    parser.add_argument('-tp', '--topics-to-pass', type=str, nargs='+')
    return parser


def pass_through_bag(in_rosbag_file, out_rosbag_file, topics_to_make_sparser=None, k=None, topics_to_pass=None):
    if topics_to_make_sparser is None:
        topics_to_make_sparser = list()
    if topics_to_pass is None:
        topics_to_pass = list()
    for ts in topics_to_make_sparser:
        if ts in topics_to_pass:
            raise RuntimeError("--topics-to-make-sparser and --topics-to-pass contain the same topic {}\n".format(ts))
    topics = topics_to_make_sparser + topics_to_pass
    if len(topics) == 0:
        return
    if k is None:
        raise RuntimeError("-k in not set\n")
    with rosbag.Bag(out_rosbag_file, 'w') as outbag:
        inbag = rosbag.Bag(in_rosbag_file)
        messages_reader = inbag.read_messages(topics=topics)
        total_number = inbag.get_message_count(topic_filters=topics)
        counters = dict()
        for topic in topics:
            counters[topic] = 1
        for topic, msg, t in tqdm(messages_reader, total=total_number):
            if topic in topics_to_make_sparser:
                if counters[topic] >= k:
                    outbag.write(topic, msg, t)
                    counters[topic] -= k
                counters[topic] += 1
            if topic in topics_to_pass:
                outbag.write(topic, msg, t)
        inbag.close()


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    pass_through_bag(**vars(args))
    
