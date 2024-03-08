import argparse
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-in', '--input-topic', required=True, type=str)
    parser.add_argument('-out', '--output-topic', required=True, type=str)
    return parser


def callback(tf: TransformStamped):
    global publisher
    odom = Odometry()
    odom.header = tf.header
    odom.child_frame_id = tf.child_frame_id
    odom.pose.pose.position.x = tf.transform.translation.x
    odom.pose.pose.position.y = tf.transform.translation.y
    odom.pose.pose.position.z = tf.transform.translation.z
    odom.pose.pose.orientation.x = tf.transform.rotation.x
    odom.pose.pose.orientation.y = tf.transform.rotation.y
    odom.pose.pose.orientation.z = tf.transform.rotation.z
    odom.pose.pose.orientation.w = tf.transform.rotation.w
    publisher.publish(odom)


if __name__ == '__main__':
    parser = build_parser()
    args, unknown_args = parser.parse_known_args()
    for i in range(len(unknown_args)-1, -1, -1):
        if unknown_args[i].startswith('__name:=') or unknown_args[i].startswith('__log:='):
            del unknown_args[i]
    if len(unknown_args) > 0:
        raise RuntimeError("Unknown args: {}".format(unknown_args))

    rospy.init_node('publish_transform_stamped_to_odometry', anonymous=True)
    rospy.Subscriber(args.input_topic, TransformStamped, callback)
    publisher = rospy.Publisher(args.output_topic, Odometry, queue_size=1)
    rospy.spin()

