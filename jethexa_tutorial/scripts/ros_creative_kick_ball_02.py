#!/usr/bin/env python3
# coding: utf8

import argparse
import time
import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from jethexa_controller import jethexa
from jethexa_sdk import misc, pid, pwm_servo
from vision_utils import get_area_max_contour, colors, fps


class ColorKickNode:
    def __init__(self, target_color, log_level=rospy.INFO):
        rospy.init_node('color_tracing_node', log_level=log_level)

        # Get the threshold range for each color from the parameter server
        color_ranges = rospy.get_param('/lab_config_manager/color_range_list', {})
        print(color_ranges)

        assert(target_color in color_ranges) # Make sure there is a record for this color
        self.target_color_name = target_color # Target color name
        self.target_color_range = color_ranges[target_color] # Target color threshold range
        rospy.loginfo("{}, {}".format(self.target_color_name, self.target_color_range))

        self.fps = fps.FPS()  # Frame rate counter

        self.jethexa = jethexa.JetHexa(self, pwm=False)
        self.jethexa.set_build_in_pose('DEFAULT_POSE', 1)
        self.pid_pitch = pid.PID(0.4, 0.01, 0.01) # pid controller that controls the pitch angle
        self.pid_yaw = pid.PID(0.4, 0.01, 0.01) # pid controller that controls the yaw angle
        self.pid_yaw_s = pid.PID(1, 0, 0) 
        self.pitch = 1500 # Pitch angle servo value 500~2500 corresponds to 0~180deg
        self.yaw = 1500 # Yaw angle servo value
        self.yaw_inc = 10
        self.count = 0

        pwm_servo.pwm_servo1.start()
        pwm_servo.pwm_servo2.start()
        pwm_servo.pwm_servo1.set_position(1500, 1000)
        pwm_servo.pwm_servo2.set_position(1200, 1000)
        rospy.sleep(1)
        self.jethexa.set_step_mode(1, 0, 10, 0, 0, 0.6, repeat=0)
        self.step = 0
        self.timestamp = time.time()


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
        self.count += 1
        try:
            img_blur = cv2.GaussianBlur(rgb_image, (3, 3), 3) # Gaussian blur
            img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # Convert to LAB space
            mask = cv2.inRange(img_lab, tuple(self.target_color_range['min']), tuple(self.target_color_range['max'])) # Binarization

            # Smooth edges, remove small blocks, merge close blocks
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

            # find the largest contour
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            max_contour_area = get_area_max_contour(contours, 50)

            # If there is a profile that meets the requirements
            if max_contour_area is not None:
                (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0]) # minimum circumcircle

                # Circle the recognized block to be tracked
                circle_color = colors.rgb[self.target_color_name] if self.target_color_name in colors.rgb else (0x55, 0x55, 0x55)
                cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)

                if self.step == 0 and self.count > 60: # let the robot face to the ball
                    # screen y-axis that controls pitch
                    if abs(center_y - (h/2)) > 30: # If the difference range is less than a certain value, there is no need to move any more.
                        self.pid_pitch.SetPoint = h / 2 # Our goal is to make the color block in the center of the picture, which is 1/2 of the pixel width of the entire picture
                        self.pid_pitch.update(center_y) # Update pid controller
                        self.pitch += self.pid_pitch.output # get pid output
                    else:
                        self.pid_pitch.clear() # reset the pid controller if it has reached the center

                    # Screen x-axis that controls yaw
                    if abs(center_x - (w / 2)) > 20:
                        self.pid_yaw.SetPoint = w / 2
                        self.pid_yaw.update(center_x)
                        self.yaw += self.pid_yaw.output
                    else:
                        self.pid_pitch.clear()
                    rospy.loginfo("pitch:{:.2f}\tyaw:{:.2f}".format(self.pitch , self.yaw))

                    # Limit the range. The movement range of the two servos has physical limits for protection
                    self.yaw = misc.set_range(self.yaw, 500, 2500)
                    self.pitch = misc.set_range(self.pitch, 1150, 2500)
                    # Set the servo angle, and the picture is about 30fps. Set the servo movement time to be similar
                    pwm_servo.pwm_servo1.set_position(self.yaw, 33)
                    pwm_servo.pwm_servo2.set_position(self.pitch, 33)

                    if abs(self.yaw - 1500) > 5:
                        self.pid_yaw_s.SetPoint = 1500
                        self.pid_yaw_s.Kp = 5
                        self.pid_yaw_s.update(self.yaw)
                        out = min(max(self.pid_yaw_s.output, -5), 5)
                        print(out)
                        self.jethexa.set_step_mode(1, 0, 10, 0, -math.radians(out), 0.6, repeat=0)
                    else:
                        self.step = 1
                        self.jethexa.set_step_mode(1, 0, 10, 0, 0, 0.6, repeat=0)
                        print(self.step)

                elif self.step == 1: # Move the ball left and right to the right position
                    if abs(center_x  - 200) > 10:
                        self.pid_yaw_s.SetPoint = 200
                        self.pid_yaw_s.Kp = 5
                        self.pid_yaw_s.update(self.yaw)
                        out = min(max(self.pid_yaw_s.output, -10), 10)
                        d = 270 if out > 0 else 90
                        self.jethexa.set_step_mode(1, abs(out), 10, math.radians(d), 0, 0.6, repeat=0)
                    else:
                        self.step = 2
                        self.jethexa.set_step_mode(1, 0, 10, 0, 0, 0.6, repeat=0)
                elif self.step == 2: # Move the ball back and forth to the right place
                    if abs(center_y - (h/2)) > 20: # If the difference range is less than a certain value, you don't need to move anymore.
                        self.pid_pitch.SetPoint = h/2 # Our goal is to make the color block in the center of the picture, which is 1/2 of the pixel width of the entire picture
                        self.pid_pitch.update(center_y) # update pid controller
                        self.pitch += self.pid_pitch.output # get pid output
                        pwm_servo.pwm_servo2.set_position(self.pitch, 33)
                    else:
                        self.pid_pitch.clear() # reset the pid controller if it has reached the center

                    if abs(self.pitch - 1200) > 10:
                        self.pid_yaw_s.SetPoint = 1200
                        self.pid_yaw_s.Kp = 5
                        self.pid_yaw_s.update(self.yaw)
                        out = min(max(self.pid_yaw_s.output, -10), 10)
                        d = 0 if out > 0 else 180
                        self.jethexa.set_step_mode(1, out, 10, math.radians(d), 0, 0.6, repeat=0)
                    else:
                        self.step = 3
                        self.jethexa.stop_running()
                        self.timestamp = time.time()
                        print(self.step)
                elif self.step == 3: # kick the ball
                    if time.time() - self.timestamp > 1:
                        try:
                            self.jethexa.run_action_set("/home/hiwonder/ActionSets/kick.d6a", repeat=1)
                        except Exception as e:
                            rospy.logerr(str(e))
                        self.timestamp = time.time()
                        self.step = 4

                elif self.step == 4:
                    if time.time() - self.timestamp > 5:
                        self.step = 0
                        self.jethexa.set_step_mode(-2, 0, 10, 0, 0, 0.6, repeat=0)
            else:
                self.step = 0
                self.pid_yaw.clear()
                self.pid_pitch.clear()
                self.pitch = 1200
                self.yaw += self.yaw_inc
                if self.yaw >= 2500 or self.yaw <= 500:
                    self.yaw_inc = -self.yaw_inc
                self.yaw = max(min(self.yaw, 2500), 500)
                pwm_servo.pwm_servo1.set_position(self.yaw, 33)
                pwm_servo.pwm_servo2.set_position(self.pitch, 33)



        except Exception as e:
            self.step = 0
            rospy.logerr(str(e))

        self.fps.update() # refresh fps stats
        self.fps.show_fps(result_image) # fps is displayed on the screen
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('target_color',  metavar="COLOR NAME", nargs='?', type=str, help="颜色名称", default='red') # the name of the color to be tracked
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    target_color = argv.target_color

    try:
        color_detect_node = ColorKickNode(target_color=target_color, log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

