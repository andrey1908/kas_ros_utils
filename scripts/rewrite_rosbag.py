from ast import parse
import rosbag
from tqdm import tqdm
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-rosbag', '--rosbag-file', type=str, required=True)
    parser.add_argument('-t', '--topics', type=str, nargs='+')
    parser.add_argument('-out', '--out-rosbag-file', type=str, required=True)
    return parser


def rewrite_rosbag(rosbag_file, out_rosbag_file, topics=None):
    with rosbag.Bag(rosbag_file, 'r') as in_bag:
        with rosbag.Bag(out_rosbag_file, 'w') as out_bag:
            messages_reader = in_bag.read_messages(topics=topics)
            total_number = in_bag.get_message_count(topic_filters=topics)
            for topic, msg, t in tqdm(messages_reader, total=total_number):
                if topic in ["/tf", "/tf_static"] and msg.transforms:
                    out_bag.write(topic, msg, msg.transforms[0].header.stamp)
                else:
                    out_bag.write(topic, msg, msg.header.stamp if msg._has_header else t)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    rewrite_rosbag(**vars(args))
