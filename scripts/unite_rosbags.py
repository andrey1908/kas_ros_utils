import rosbag
import argparse
from tqdm import tqdm


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-rosbags', '--rosbag-files', required=True, nargs='+', type=str)
    parser.add_argument('-t', '--topics', nargs='+', type=str)
    parser.add_argument('-out', '--out-rosbag-file', required=True, type=str)
    return parser


def unite_rosbags(rosbag_files, out_rosbag_file, topics=None):
    with rosbag.Bag(out_rosbag_file, 'w') as out_bag:
        for rosbag_file in tqdm(rosbag_files):
            with rosbag.Bag(rosbag_file, 'r') as in_bag:
                messages_reader = in_bag.read_messages(topics=topics)
                total_number = in_bag.get_message_count(topic_filters=topics)
                for topic, msg, t in tqdm(messages_reader, total=total_number):
                    out_bag.write(topic, msg, t)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()

