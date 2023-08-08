#!/usr/bin/env python

import argparse
import sys
import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from ros_numpy.point_cloud2 import array_to_pointcloud2
from cv_bridge import CvBridge
import numpy as np
from time_measurer import TimeMeasurer

try:  # prefer using torch since it's faster
    import torch
except ImportError:
    from scipy.ndimage import minimum_filter
    print(
        "\033[1;93m"
        "Could not import torch module. "
        "Will use scipy.ndimage instead, but it's slower. "
        "Recommend to install torch for better performance."
        "\033[0m")


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-camera-info', '--camera-info-topic', type=str, required=True)
    parser.add_argument('-depth', '--depth-topic', type=str, required=True)
    parser.add_argument('--pool-size', type=int, default=8)
    parser.add_argument('-out', '--out-topic', type=str, required=True)
    return parser


class DepthToPointCloud:
    def __init__(self, pool_size=8):
        self.pool_size = pool_size

        self.bridge = CvBridge()

        if 'torch' in sys.modules:
            self.pool_fn = self._pool_depth_with_torch
        else:
            self.pool_fn = self._pool_depth_with_scipy

        self.convertion_tm = TimeMeasurer("  convertion")
        self.total_tm = TimeMeasurer("total", end="\n")
        self.tms = [self.convertion_tm, self.total_tm]

    @staticmethod
    def _get_depth_factor(depth):
        if depth.dtype == np.uint16:
            return 0.001
        if depth.dtype == float:
            return 1
        raise RuntimeError(f"Unknown depth dtype: {depth.dtype}")

    def _pool_depth_with_torch(self, depth):
        depth = np.expand_dims(depth, axis=0)
        max_pool_fn = torch.nn.MaxPool2d(self.pool_size)
        pooled = -max_pool_fn(-torch.from_numpy(depth)).numpy().squeeze()
        return pooled

    def _pool_depth_with_scipy(self, depth):
        pooled = minimum_filter(depth, size=self.pool_size,
            origin=-(self.pool_size // 2), mode='constant', cval=np.inf)
        pooled = pooled[::self.pool_size, ::self.pool_size]
        return pooled

    def depth_to_point_cloud(self, camera_info_msg: CameraInfo, depth_msg: Image):
        with self.total_tm:
            assert camera_info_msg.header.frame_id == depth_msg.header.frame_id
            assert all(x == 0 for x in camera_info_msg.D), "Only rect depth images are supported"
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            with self.convertion_tm:
                factor = DepthToPointCloud._get_depth_factor(depth)
                depth = depth * factor

                invalid = (depth <= 0) | ~np.isfinite(depth)
                depth[invalid] = np.inf

                pooled = self.pool_fn(depth)
                K = np.array(camera_info_msg.K).reshape(3, 3)
                K /= self.pool_size
                K[2, 2] = 1
                fx = K[0, 0]
                fy = K[1, 1]
                cx = K[0, 2]
                cy = K[1, 2]

                valid = np.isfinite(pooled)
                z = pooled[valid]
                v, u = np.where(valid)
                x = (u - cx) / fx * z
                y = (v - cy) / fy * z

                x = x.astype(np.float32)
                y = y.astype(np.float32)
                z = z.astype(np.float32)

            points = np.rec.fromarrays([x, y, z], names=['x', 'y', 'z'])
            pc_msg = array_to_pointcloud2(points,
                stamp=depth_msg.header.stamp, frame_id=depth_msg.header.frame_id)
            return pc_msg


class DepthToPointCloud_node(DepthToPointCloud):
    def __init__(self, camera_info_topic, depth_topic, out_topic, pool_size=8):
        super().__init__(pool_size=pool_size)

        self.camera_info_topic = camera_info_topic
        self.depth_topic = depth_topic
        self.out_topic = out_topic

        self.camera_info_sub = message_filters.Subscriber(camera_info_topic, CameraInfo)
        self.depth_sub = message_filters.Subscriber(depth_topic, Image)
        self.sync_sub = message_filters.TimeSynchronizer(
            [self.camera_info_sub, self.depth_sub], 10)
        self.sync_sub.registerCallback(self.callback)

        self.pc_pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=10)

    def callback(self, camera_info_msg: CameraInfo, depth_msg: Image):
        pc_msg = self.depth_to_point_cloud(camera_info_msg, depth_msg)
        self.pc_pub.publish(pc_msg)


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
    TimeMeasurer.hide = True

    print("Spinning...")
    rospy.spin()

    print()
    for tm in node.tms:
        tm.print()
