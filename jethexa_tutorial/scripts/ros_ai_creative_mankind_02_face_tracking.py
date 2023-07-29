#!/usr/bin/env python3
"""
这个程序实现了人脸追踪功能
运行现象：桌面显示识别结果画面， 将识别到的人脸在画面中框出
        机器人的头部云台跟随最靠近画面中心的人脸移动

"""
import cv2
import gc
import rospy
import time
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps, box_center, distance
from utils import show_faces, mp_face_location
from jethexa_sdk import pid, pwm_servo


class FaceTrackingNode:
    def __init__(self):
        rospy.init_node("face_tracking_node", log_level=rospy.INFO) # initialize node

        # face detector
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            # model_selection=0,
            min_detection_confidence=0.5,
        )
        # self.drawing = mp.solutions.drawing_utils

        self.pitch, self.yaw = 1500, 1500 # Pan-tilt pitch angle and yaw angle
        self.pid_pitch = pid.PID(25, 0, 4) # Pan-tilt pitch angle pid 
        self.pid_yaw = pid.PID(30, 0, 8) # Pan-tilt yaw angle pid
        pwm_servo.pwm_servo1.start()
        pwm_servo.pwm_servo2.start()
        pwm_servo.pwm_servo1.set_position(self.pitch, 1000)
        pwm_servo.pwm_servo2.set_position(self.yaw, 1000)

        self.fps = fps.FPS() # Frame rate counter
        self.detected_face = 0 # The number of frames in which the face was continuously recognized

        # Subscribe and publish images
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=2)
        print(self.camera_rgb_prefix)
        self.timestamp = time.time()

    def image_callback(self, ros_image):
        rospy.logdebug('Received an image! ')
        # raw RGB image
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) 
        result_image = np.copy(rgb_image)

        results = self.face_detector.process(rgb_image) # recognize the human face in the image and the face key points
        boxes, keypoints = mp_face_location(results, rgb_image) # Get the output data of face recognition and convert the normalized data to pixel coordinates
        o_h, o_w = rgb_image.shape[:2]

        if len(boxes) > 0:
            self.detected_face += 1 
            self.detected_face = min(self.detected_face, 20) # make the count always not greater than 20

            # If the face is recognized in 5 consecutive frames, start tracking to avoid misrecognition
            if self.detected_face >= 5:
                center = [box_center(box) for box in boxes] # Calculate the center coordinates of all faces
                dist = [distance(c, (o_w / 2, o_h / 2)) for c in center] # Calculate the distance from the center coordinates of all faces to the center of the screen
                face = min(zip(boxes, center, dist), key=lambda k: k[2]) # Find the face with the smallest distance from the center of the frame

                # Calculate the x-axis distance of the face to be tracked from the center of the screen, and normalize (-1~+1)
                c_x, c_y = face[1]
                dist_x = 1.0 - c_x / (o_w / 2) #Simplified by this (self.detect_w / 2.0 - c_x) / (self.detect_w / 2)
                dist_y = 1.0 - c_y / (o_h / 2) #Simplified by this (self.detect_h / 2.0 - c_y) / (self.detect_h / 2)

                if abs(dist_y) > 0.3:
                    self.pid_pitch.SetPoint = 0
                    self.pid_pitch.update(dist_y) # Update pitch pid controller
                    # Get a new pitch angle and limit the range of motion
                    self.pitch = min(max(self.pitch - self.pid_pitch.output, 1200), 2500)

                if abs(dist_x) > 0.2:
                    self.pid_yaw.SetPoint = 0
                    self.pid_yaw.update(dist_x) # Update yaw angle pid controller
                    # Get new yaw angle and limit range of motion
                    self.yaw = min(max(self.yaw - self.pid_yaw.output, 500),  2500)

                pwm_servo.pwm_servo1.set_position(self.yaw, 20)
                pwm_servo.pwm_servo2.set_position(self.pitch, 20)

        else: # process the face not recognized
            if self.detected_face > 0:
                self.detected_face -= 1
            else:
                self.pid_pitch.clear()
                self.pid_yaw.clear()

        result_image = show_faces(rgb_image, result_image, boxes, keypoints) # Display the recognized face and face key points in the picture
        self.fps.update() # Update fsp stats
        result_image = self.fps.show_fps(result_image) # fps is displayed on the screen
        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        if time.time() - self.timestamp > 0.3:
            self.timestamp = time.time()
            gc.collect()


if __name__ == "__main__":
    try:
        face_tracking_node = FaceTrackingNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

