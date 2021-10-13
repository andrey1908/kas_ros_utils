#!/usr/bin/env python3
import rospy
import rostopic
import roslib
from geometry_msgs.msg import PoseStamped
from sklearn.linear_model import LinearRegression
import numpy as np
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--topic', required=True, type=str)
    return parser


def read_message_latency(msg):
    global measurements
    now = rospy.get_time()
    latency = now - msg.header.stamp.to_sec()
    measurements.append(latency)


def measure_latency(topic):
    data_type = rostopic.get_topic_type(topic, blocking=True)[0]
    data_class = roslib.message.get_message_class(data_type)
    rospy.Subscriber(topic, data_class, read_message_latency)
    print("Reading messages...")
    rospy.spin()
    global measurements
    print()
    print("Number of measurements: {}".format(len(measurements)))
    if len(measurements) > 0:
        reg = LinearRegression().fit(np.array(list(range(len(measurements)))).reshape(-1, 1), measurements)
        print("Total measured latency: {:.6f}".format(sum(measurements)))
        print("Average latency: {:.6f}".format(sum(measurements) / len(measurements)))
        print("Max latency: {:.6f}".format(max(measurements)))
        print("Coef: {:.9f}".format(reg.coef_[0]))


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    
    rospy.init_node('measure_latency', anonymous=True)
    measurements = list()
    measure_latency(**vars(args))

