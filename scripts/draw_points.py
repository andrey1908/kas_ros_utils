import argparse
import rospy
import tf2_ros
import rostopic
import message_filters
from ros_numpy.geometry import transform_to_numpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from cv_bridge import CvBridge
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
import numpy as np
from kas_utils.visualization import draw_points


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-camera-info', '--camera-info-topic', type=str, required=True)
    parser.add_argument('-image', '--image-topic', type=str, required=True)
    parser.add_argument('-point-cloud', '--point-cloud-topic', type=str, required=True)
    parser.add_argument('-out-image', '--out-image-topic', type=str, required=True)
    return parser


def callback(image_msg, point_cloud_msg):
    global pub, bridge, tf_buffer, K, D

    if image_msg._type == "sensor_msgs/Image":
        image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    elif image_msg._type == "sensor_msgs/CompressedImage":
        image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    else:
        raise RuntimeError("Unkown message type")
    points = pointcloud2_to_xyz_array(point_cloud_msg)

    try:
        tf = tf_buffer.lookup_transform(image_msg.header.frame_id, point_cloud_msg.header.frame_id,
            image_msg.header.stamp, timeout=rospy.Duration(0.1))
    except tf2_ros.ExtrapolationException as ex:
        print(str(ex))
        return
    tf_mat = transform_to_numpy(tf.transform)
    R = tf_mat[:3, :3]
    t = tf_mat[:3, 3]
    points = np.matmul(R, points.T).T + t

    draw_points(image, K, D, points)

    out_image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    out_image_msg.header = image_msg.header
    pub.publish(out_image_msg)


if __name__ == "__main__":
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    rospy.init_node("draw_points")

    pub = rospy.Publisher(args.out_image_topic, Image, queue_size=10)

    bridge = CvBridge()

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    print("Waiting for camera info message...")
    camera_info_msg = rospy.wait_for_message(args.camera_info_topic, CameraInfo)
    K = np.array(camera_info_msg.K).reshape(3, 3)
    D = np.array(camera_info_msg.D)

    image_topic_type, _, _ = rostopic.get_topic_class(args.image_topic)
    image_sub = message_filters.Subscriber(args.image_topic, image_topic_type,
        queue_size=1, buff_size=2 ** 24)
    point_cloud_sub = message_filters.Subscriber(args.point_cloud_topic, PointCloud2,
        queue_size=1, buff_size=2 ** 24)
    sync_sub = message_filters.ApproximateTimeSynchronizer([image_sub, point_cloud_sub],
        queue_size=50, slop=0.1)
    sync_sub.registerCallback(callback)

    print("Spinning...")
    rospy.spin()
