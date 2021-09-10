#!/usr/bin/env python3
import rosbag
import numpy as np
from tqdm import tqdm
import argparse
import matplotlib.pyplot as plt


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-rosbag', '--rosbag-file', required=True, type=str)
    parser.add_argument('-t', '--topics', required=True, type=str, nargs='+')
    parser.add_argument('--check-publishing-time', action='store_true')
    parser.add_argument('-w', '--window-size', type=float, default=1.)
    return parser


class TimestampsKeeper:
    def __init__(self):
        self.timestamps = dict()
        self.sorted_timestamps = dict()
        self.is_sorted = True

    def add(self, topic, timestamp):
        self.is_sorted = False
        if topic not in self.timestamps.keys():
            self.timestamps[topic] = list()
        self.timestamps[topic].append(timestamp)

    def sort(self):
        self.sorted_timestamps.clear()
        for topic in self.timestamps.keys():
            self.sorted_timestamps[topic] = sorted(self.timestamps[topic])
        self.is_sorted = True

    def count_msgs_in_time_window(self, window_size):
        if not self.is_sorted:
            raise(RuntimeError)
        result = dict()
        for topic in self.sorted_timestamps.keys():
            result[topic] = list()
            timestamps = self.sorted_timestamps[topic]
            if len(timestamps) == 0:
                continue
            current_time = timestamps[0]
            first_msg = 0
            while first_msg < len(timestamps):
                msgs_number = 0
                while timestamps[first_msg + msgs_number] < current_time + window_size:
                    msgs_number += 1
                    if first_msg + msgs_number == len(timestamps):
                        break
                result[topic].append(msgs_number)
                if first_msg + msgs_number == len(timestamps):
                    result[topic].pop()  # TODO: correct number of messages for the last window instead of popping
                    pass
                first_msg += msgs_number
                current_time += window_size
        return result

    def get_max_gaps(self):
        if not self.is_sorted:
            raise(RuntimeError)
        result = dict()
        for topic in self.sorted_timestamps.keys():
            timestamps = self.sorted_timestamps[topic]
            gaps = np.array(timestamps + [timestamps[-1]]) - np.array([timestamps[0]] + timestamps)
            result[topic] = max(gaps)
        return result


def check_messages_timestamps(rosbag_file, topics, check_publishing_time=False, window_size=1.):
    with rosbag.Bag(rosbag_file) as bag:
        timestamps_keeper = TimestampsKeeper()
        for topic, msg, t in tqdm(bag.read_messages(topics=topics), total=bag.get_message_count(topic_filters=topics)):
            if topic not in ['/tf', '/tf_static']:
                if check_publishing_time:
                    timestamp = t.to_sec()
                else:
                    timestamp = msg.header.stamp.to_sec()
                timestamps_keeper.add(topic, timestamp)
    timestamps_keeper.sort()

    result = timestamps_keeper.count_msgs_in_time_window(window_size)
    for topic in topics:
        plt.plot(np.arange(len(result[topic])) * window_size, np.array(result[topic]) / window_size)
        plt.title(topic)
        plt.xlabel('time, s')
        plt.ylabel('{} frequency, hz'.format('publishing' if check_publishing_time else 'timestamp'))
        plt.show()

    result = timestamps_keeper.get_max_gaps()
    for topic in topics:
        print('Max gap in {}: {:.3} s'.format(topic, result[topic]))


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    check_messages_timestamps(args.rosbag_file, args.topics, args.check_publishing_time, args.window_size)
    