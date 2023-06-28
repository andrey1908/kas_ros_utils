#!/usr/bin/env python
import argparse
import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
from tqdm import tqdm


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-video', '--video-file', required=True, type=str)
    parser.add_argument('-t', '--out-topic', required=True, type=str)
    parser.add_argument('--frame-id', type=str, default="camera")
    parser.add_argument('--start-bag-time', type=float, default=0.)
    parser.add_argument('--compress-images', action='store_true')
    parser.add_argument('-out', '--out-file', required=True, type=str)
    return parser


def video_to_rosbag(video_file, out_topic, out_file, frame_id="camera", start_bag_time=0.,
        compress_images=False):
    video = cv2.VideoCapture(video_file)
    assert video.isOpened()

    fps = video.get(cv2.CAP_PROP_FPS)
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    stamp = rospy.Time.from_sec(start_bag_time)
    frame_duration = rospy.Duration.from_sec(1 / fps)
    bridge = CvBridge()
    pbar = tqdm(total=total_frames)
    with rosbag.Bag(out_file, 'w') as bag:
        ret = True
        while ret:
            ret, image = video.read()
            if ret == True:
                if compress_images:
                    image_msg = bridge.cv2_to_compressed_imgmsg(image)
                else:
                    image_msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
                image_msg.header.frame_id = frame_id
                image_msg.header.stamp = stamp
                bag.write(out_topic, image_msg, stamp)
                stamp += frame_duration
                pbar.update()
    pbar.close()
    video.release()


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    video_to_rosbag(**vars(args))
