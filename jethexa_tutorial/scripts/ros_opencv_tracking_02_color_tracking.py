#!/usr/bin/env python3
# coding: utf8

import argparse
import sys
import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from jethexa_sdk import misc, pid, pwm_servo
from vision_utils import get_area_max_contour, colors, fps


class ColorTrackingNode:
    def __init__(self, target_color, log_level=rospy.INFO):
        rospy.init_node('color_tracing_node', log_level=log_level)

        # Get the threshold range for each color from the parameter server
        color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {})
        print(color_ranges)

        assert(target_color in color_ranges) # Make sure there is a record for this color
        self.target_color_name = target_color # target color name
        self.target_color_range = color_ranges[target_color] # target color threshold range
        rospy.loginfo("{}, {}".format(self.target_color_name, self.target_color_range))

        self.fps = fps.FPS()  # Frame rate counter

        self.pid_pitch = pid.PID(0.4, 0.01, 0.01) # pid controller that controls the pitch angle
        self.pid_yaw = pid.PID(0.5, 0.01, 0.01) # pid controller that controls the yaw angle
        self.pitch = 1500 # Pitch angle servo value 500~2500 corresponds to 0~180deg
        self.yaw = 1500 # Yaw angle servo value

        pwm_servo.pwm_servo1.start()
        pwm_servo.pwm_servo2.start()
        pwm_servo.pwm_servo1.set_position(1500, 1000)
        pwm_servo.pwm_servo2.set_position(1500, 1000)

        # image topic
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        rospy.loginfo("object tracking node created")

    def image_callback(self, ros_image: Image):
        # rospy.logdebug('Received an image! ')
        # Convert ros format image to opencv format
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # raw RGB image
        rgb_image = cv2.resize(rgb_image, (320, 180), cv2.INTER_NEAREST) # scale the image
        result_image = np.copy(rgb_image) # Make a copy for display of results in case the image was modified during processing
        
        h, w = rgb_image.shape[:2]
        try:
            img_blur = cv2.GaussianBlur(rgb_image, (3, 3), 3) # Gaussian blur
            img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # Convert to LAB space
            mask = cv2.inRange(img_lab, tuple(self.target_color_range['min']), tuple(self.target_color_range['max'])) # Binarization

            # Smooth edges, remove small blocks, and merge close blocks
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

            # find the largest contour
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            max_contour_area = get_area_max_contour(contours, 50)

            # If there is a profile that meets the requirements
            if max_contour_area is not None:
                (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0]) # minimum circumcircle

                # circle the recognized block to tracked
                circle_color = colors.rgb[self.target_color_name] if self.target_color_name in colors.rgb else (0x55, 0x55, 0x55)
                cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)

                # screen y-axis that controls pitch
                if abs(center_y - (h / 2)) > 30: # If the difference range is less than a certain value, there is no need to move any more.
                    self.pid_pitch.SetPoint = h / 2 # Our goal is to make the color block in the center of the picture, which is 1/2 of the pixel width of the entire picture
                    self.pid_pitch.update(center_y) # update pid controller
                    self.pitch += self.pid_pitch.output # get pid output
                else:
                    self.pid_pitch.clear() # reset the pid controller if it has reached the center

                # screen x-axis that controls yaw
                if abs(center_x - (w / 2)) > 20:
                    self.pid_yaw.SetPoint = w / 2
                    self.pid_yaw.update(center_x)
                    self.yaw += self.pid_yaw.output
                else:
                    self.pid_pitch.clear()
                rospy.loginfo("pitch:{:.2f}\tyaw:{:.2f}".format(self.pitch , self.yaw))

                # Limit the range. The movement range of the two servos has physical limits for protection. 
                self.yaw = misc.set_range(self.yaw, 500, 2500)
                self.pitch = misc.set_range(self.pitch, 1300, 2500)

                # Set the servo angle, and the picture is about 30fps. Set the servo movement time to be similar
                pwm_servo.pwm_servo1.set_position(self.yaw, 40) 
                pwm_servo.pwm_servo2.set_position(self.pitch, 40)
            else:
                self.pid_yaw.clear()
                self.pid_pitch.clear()

        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # refresh fps stats
        self.fps.show_fps(result_image) # fps is displayed on the screen
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('target_color',  metavar="COLOR NAME", nargs='?', type=str, help="颜色名称", default='red') # the name of the color to tracked
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    target_color = argv.target_color

    try:
        color_detect_node = ColorTrackingNode(target_color=target_color, log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

