#!/usr/bin/env python
import rospy
import rosbag
import numpy as np
from tqdm import tqdm
import argparse
import matplotlib.pyplot as plt


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-rosbag', '--rosbag-file', required=True, type=str)
    parser.add_argument('-t', '--topics', type=str, nargs='+')
    parser.add_argument('-w', '--window-size', type=float, default=1.)
    return parser


class TimestampsKeeper:
    def __init__(self):
        self.header_timestamps = dict()
        self.publishing_times = dict()
        self.sorted_header_timestamps = dict()
        self.sorted_publishing_times = dict()
        self.is_sorted = True

    def add_header_timestamp(self, topic, header_timestamp):
        self.is_sorted = False
        if topic not in self.header_timestamps.keys():
            self.header_timestamps[topic] = list()
        self.header_timestamps[topic].append(header_timestamp)

    def add_publishing_time(self, topic, publishing_time):
        self.is_sorted = False
        if topic not in self.publishing_times.keys():
            self.publishing_times[topic] = list()
        self.publishing_times[topic].append(publishing_time)

    def sort(self):
        self.sorted_header_timestamps.clear()
        self.sorted_publishing_times.clear()
        for topic in self.header_timestamps.keys():
            self.sorted_header_timestamps[topic] = sorted(self.header_timestamps[topic])
        for topic in self.publishing_times.keys():
            self.sorted_publishing_times[topic] = sorted(self.publishing_times[topic])
        self.is_sorted = True

    def count_msgs_in_time_window(self, window_size):
        if not self.is_sorted:
            raise RuntimeError()
        msgs_counts = dict()
        msgs_counts['header_timestamps'] = dict()
        msgs_counts['publishing_times'] = dict()
        window_size_ros = rospy.Duration.from_sec(window_size)
        for topic in self.sorted_header_timestamps.keys():
            timestamps = self.sorted_header_timestamps[topic]
            msgs_counts['header_timestamps'][topic] = self.count_msgs_in_time_window_for_sorted_timestamps(timestamps, window_size_ros)
        for topic in self.sorted_publishing_times.keys():
            timestamps = self.sorted_publishing_times[topic]
            msgs_counts['publishing_times'][topic] = self.count_msgs_in_time_window_for_sorted_timestamps(timestamps, window_size_ros)
        return msgs_counts

    def count_msgs_in_time_window_for_sorted_timestamps(self, timestamps, window_size_ros):
        if not self.is_ascending(timestamps):
            raise RuntimeError()
        msgs_count = list()
        if len(timestamps) == 0:
            return msgs_count
        current_time = timestamps[0]
        first_msg = 0
        while first_msg < len(timestamps):
            msgs_number = 0
            while timestamps[first_msg + msgs_number] < current_time + window_size_ros:
                msgs_number += 1
                if first_msg + msgs_number == len(timestamps):
                    break
            msgs_count.append(msgs_number)
            if first_msg + msgs_number == len(timestamps):
                msgs_count.pop()  # TODO: correct number of messages for the last window instead of popping
                pass
            first_msg += msgs_number
            current_time += window_size_ros
        return msgs_count

    def is_ascending(self, timestamps):
        previous = timestamps[0]
        for current in timestamps:
            if previous > current:
                return False
            previous = current
        return True
        
    def get_max_gaps(self):
        if not self.is_sorted:
            raise RuntimeError()
        max_gaps = dict()
        max_gaps['header_timestamps'] = dict()
        max_gaps['publishing_times'] = dict()
        for topic in self.sorted_header_timestamps.keys():
            timestamps = self.sorted_header_timestamps[topic]
            gaps = np.array(timestamps + [timestamps[-1]]) - np.array([timestamps[0]] + timestamps)
            max_gap = max(gaps)
            max_gaps['header_timestamps'][topic] = max_gap
        for topic in self.sorted_publishing_times.keys():
            timestamps = self.sorted_publishing_times[topic]
            gaps = np.array(timestamps + [timestamps[-1]]) - np.array([timestamps[0]] + timestamps)
            max_gap = max(gaps)
            max_gaps['publishing_times'][topic] = max_gap
        return max_gaps

    def get_publishing_times_and_header_timestamps_differences(self):
        publishing_times_and_header_timestamps_differences = dict()
        topics = set(self.header_timestamps.keys())
        for topic in self.publishing_times.keys():
            if topic not in topics:
                continue
            if len(self.header_timestamps[topic]) != len(self.publishing_times[topic]):
                raise RuntimeError()
            header_timestamps = np.array(self.header_timestamps[topic])
            publishing_times = np.array(self.publishing_times[topic])
            difference = publishing_times - header_timestamps
            publishing_times_and_header_timestamps_differences[topic] = difference
        return publishing_times_and_header_timestamps_differences

    def get_time_ranges(self):
        if not self.is_sorted:
            raise RuntimeError()
        time_ranges = dict()
        time_ranges['header_timestamps'] = dict()
        time_ranges['publishing_times'] = dict()
        for topic in self.sorted_header_timestamps.keys():
            timestamps = self.sorted_header_timestamps[topic]
            time_ranges['header_timestamps'][topic] = timestamps[-1] - timestamps[0]
        for topic in self.sorted_publishing_times.keys():
            timestamps = self.sorted_publishing_times[topic]
            time_ranges['publishing_times'][topic] = timestamps[-1] - timestamps[0]
        return time_ranges


def check_messages_timestamps(rosbag_file, topics=None, window_size=1.):
    with rosbag.Bag(rosbag_file) as bag:
        if topics is None:
            topics = list(bag.get_type_and_topic_info().topics.keys())
        timestamps_keeper = TimestampsKeeper()
        for topic, msg, t in tqdm(bag.read_messages(topics=topics), total=bag.get_message_count(topic_filters=topics)):
            if topic not in ['/tf', '/tf_static']:
                timestamps_keeper.add_header_timestamp(topic, msg.header.stamp)
                timestamps_keeper.add_publishing_time(topic, t)
    timestamps_keeper.sort()

    max_gaps = timestamps_keeper.get_max_gaps()
    time_ranges = timestamps_keeper.get_time_ranges()
    msgs_counts = timestamps_keeper.count_msgs_in_time_window(window_size)
    publishing_times_and_header_timestamps_differences = timestamps_keeper.get_publishing_times_and_header_timestamps_differences()
    for topic in topics:
        print('Max gap in {} from header timestamps: {:.3} s'.format(topic, max_gaps['header_timestamps'][topic].to_sec()))
        print('Header timestamps range for {}: {:.3} s'.format(topic, time_ranges['header_timestamps'][topic].to_sec()))
        msgs_count_from_header_timestamps = msgs_counts['header_timestamps'][topic]
        plt.plot(np.arange(len(msgs_count_from_header_timestamps)) * window_size, np.array(msgs_count_from_header_timestamps) / window_size)
        plt.title(topic)
        plt.xlabel('header time (shifted to zero), s')
        plt.ylabel('frequency from header timestamps, hz')
        plt.show()

        print('Max gap in {} from publishing times: {:.3} s'.format(topic, max_gaps['publishing_times'][topic].to_sec()))
        print('Publishing times range for {}: {:.3} s'.format(topic, time_ranges['publishing_times'][topic].to_sec()))
        msgs_count_from_publishing_times = msgs_counts['publishing_times'][topic]
        plt.plot(np.arange(len(msgs_count_from_publishing_times)) * window_size, np.array(msgs_count_from_publishing_times) / window_size)
        plt.title(topic)
        plt.xlabel('publishing time (shifted to zero), s')
        plt.ylabel('frequency from publishing times, hz')
        plt.show()

        difference = publishing_times_and_header_timestamps_differences[topic]
        difference = list(map(rospy.Duration.to_sec, difference))
        plt.plot(np.arange(len(difference)), np.array(difference))
        plt.title(topic)
        plt.xlabel('message number')
        plt.ylabel('difference between publishing times and header timestamps, s')
        plt.show()

        print()


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    check_messages_timestamps(**vars(args))
