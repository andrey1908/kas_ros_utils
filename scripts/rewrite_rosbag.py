from ast import parse
import rosbag
from tqdm import tqdm
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-in', '--in-rosbag-file', type=str, required=True)
    parser.add_argument('-t', '--topics', type=str, nargs='+')
    parser.add_argument('-out', '--out-rosbag-file', type=str, required=True)
    return parser


def rewrite_rosbag(in_rosbag_file, out_rosbag_file, topics=None):
    with rosbag.Bag(out_rosbag_file, 'w') as outbag:
        inbag = rosbag.Bag(in_rosbag_file)
        if topics is None:
            topics = list(inbag.get_type_and_topic_info()[1].keys())
        messages_reader = inbag.read_messages(topics=topics)
        total_number = inbag.get_message_count(topic_filters=topics)
        for topic, msg, t in tqdm(messages_reader, total=total_number):
            # This also replaces tf timestamps under the assumption 
            # that all transforms in the message share the same timestamp
            if topic in ["/tf", "/tf_static"] and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
        inbag.close()


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    rewrite_rosbag(**vars(args))
