#!/usr/bin/env python

import argparse
import rospy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from ros_numpy.point_cloud2 import array_to_pointcloud2
from cv_bridge import CvBridge
import numpy as np
from kas_utils.time_measurer import TimeMeasurer
from kas_utils.depth_to_point_cloud import DepthToPointCloud


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-depth-info', '--depth-info-topic', type=str, required=True)
    parser.add_argument('-depth', '--depth-topic', type=str, required=True)
    parser.add_argument('--pool-size', type=int, default=8)
    parser.add_argument('-out', '--out-topic', type=str, required=True)
    return parser


class DepthToPointCloud_node(DepthToPointCloud):
    def __init__(self, depth_info_topic, depth_topic, out_topic, pool_size=8):
        print("Waiting for depth info message...")
        depth_info_msg = rospy.wait_for_message(depth_info_topic, CameraInfo)
        K = np.array(depth_info_msg.K).reshape(3, 3)
        D = np.array(depth_info_msg.D)
        assert np.all(D == 0), "Distorted depth images are not supported."

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        super().__init__(fx, fy, cx, cy, pool_size)

        self.depth_topic = depth_topic
        self.out_topic = out_topic

        self.point_cloud_pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=10)

        self.bridge = CvBridge()

        self.convertion_tm = TimeMeasurer("  convertion")
        self.total_tm = TimeMeasurer("total")

    def start(self):
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.callback,
            queue_size=1, buff_size=2 ** 24)

    def callback(self, depth_msg: Image):
        with self.total_tm:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            with self.convertion_tm:
                point_cloud = self.convert(depth)
                assert point_cloud.dtype == np.float32
                dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
                point_cloud = point_cloud.view(dtype).squeeze()
            point_cloud_msg = array_to_pointcloud2(point_cloud,
                stamp=depth_msg.header.stamp, frame_id=depth_msg.header.frame_id)
            self.point_cloud_pub.publish(point_cloud_msg)


if __name__ == "__main__":
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    rospy.init_node("depth_to_point_cloud")
    node = DepthToPointCloud_node(**vars(args))
    node.start()

    print("Spinning...")
    rospy.spin()

    print()
