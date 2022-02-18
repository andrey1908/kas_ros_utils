#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-topic', '--topic-name', required=True, type=str)
    parser.add_argument('-out', '--out-file', required=True, type=str)
    return parser


def draw_occupancy_grid(occupancy_grid_mgs, frame=None):
    width = occupancy_grid_mgs.info.width
    height = occupancy_grid_mgs.info.height
    image = np.zeros((height, width, 3), np.uint8)
    data_iter = 0
    for j in range(height-1, -1, -1):
        for i in range(width):
            occ_grid_value = occupancy_grid_mgs.data[data_iter]
            if occ_grid_value == -1:
                pixel_value = 100
            else:
                pixel_value = int((100 - occ_grid_value) * 255 / 100)
            image[j, i] = (pixel_value, pixel_value, pixel_value)
            data_iter += 1
    
    k = 1
    if frame:
        k = min(frame[0] / width, frame[1] / height)
        image = cv2.resize(image, (int(width * k), int(height * k)))
    return image, k


def save_occupancy_grid(topic_name, out_file):
    occupancy_grid_mgs = rospy.wait_for_message(topic_name, OccupancyGrid)
    image, _ = draw_occupancy_grid(occupancy_grid_mgs)
    cv2.imwrite(out_file, image)


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()

    rospy.init_node('save_occupancy_grid')
    save_occupancy_grid(**vars(args))
