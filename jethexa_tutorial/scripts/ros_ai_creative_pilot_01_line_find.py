#!/usr/bin/env python3

import sys
import argparse
import math
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from jethexa_controller import client
from vision_utils import fps, get_area_max_contour


class LineFinderNode:
    def __init__(self, target_color):
        rospy.init_node("line_find_node") #Initialize the ros node

        # Get color threshold range from parameter server by color name
        color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {}) 
        assert(target_color in color_ranges)  # Make sure the specified color name has a corresponding threshold
        self.target_color_range = color_ranges[target_color]
        self.target_color_name = target_color
        rospy.loginfo("finding color_name: {}, range: {}".format(self.target_color_name, self.target_color_range))

        self.fps = fps.FPS() # frame counter

        # Divide the screen into three segments and find the color of the line in the three segments
        self.rois = ((330, 360, 0, 640, 0.7), (260, 290, 0, 640, 0.3), (180, 220, 0, 640, 0.1))

        # Subscribe to camera image
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)


    def image_callback(self, ros_image: Image):
        #rospy.loginfo('Received an image! ')
        # Convert the received ros format image to opencv format
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image) # Copy the original image as the background of the result output
        try:
            # raverse all areas of interest
            for roi in self.rois:
                blob = rgb_image[roi[0]:roi[1], roi[2]:roi[3]] # Capture the part of interest from the big picture
                img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB) # Convert to lab space
                img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3) # Gaussian blur

                #Binarize based on target color threshold range
                mask = cv2.inRange(img_blur, tuple(self.target_color_range['min']), tuple(self.target_color_range['max']))

                # Open and close operation, smooth edges, remove too small color blocks, merge adjacent color blocks
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) 
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

                # find out the contour
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
                max_contour_area = get_area_max_contour(contours, 30) # find the largest contour

                if max_contour_area is not None:
                    rect = cv2.minAreaRect(max_contour_area[0]) # Get the smallest bounding rectangle of the largest contour
                    box = np.int0(cv2.boxPoints(rect))
                    for j in range(4):
                        box[j, 1] = box[j, 1] + roi[0]
                    cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # Draw a rectangle with four points

                    # Get the corners of the rectangle
                    pt1_x, pt1_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    # center point of line
                    line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
                    cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)

        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # refresh frame counter
        self.fps.show_fps(result_image) # Display the number of frames in the result screen
        result_image =  cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros will pass in some of its own parameters when starting the py file, and myargv will remove these and return the original parameters
    parser = argparse.ArgumentParser() # parameter parser
    parser.add_argument('target_color', metavar='color', nargs='?', type=str, help="颜色名称如 red", default="red") # Add parameters to parse
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    try:
        line_finder_node = LineFinderNode(argv.target_color)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


