#!/usr/bin/env python3
"""
这个程序实现了人体骨架识别
运行现象：桌面显示识别结果画面， 显示人体骨架连线
"""
import cv2
import rospy
from sensor_msgs.msg import Image
from vision_utils import fps
import numpy as np
import mediapipe as mp


class PoseNode:
    def __init__(self):
        rospy.init_node("mankind_pose_node", anonymous=True)

        # Instantiate a body recognizer
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.2
        )
        self.drawing = mp.solutions.drawing_utils # Results drawing tool

        self.fps = fps.FPS() # frame rate counter
        # Subscribe to images
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        # rospy.logdebug('Received an image! ')
        # Convert ros format screen to opencv format
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # raw RGB image
        rgb_image = cv2.flip(rgb_image, 1)  # mirror image. You can view the effect directly on the screen and the camera 
        result_image = np.copy(cv2.resize(rgb_image, (int(ros_image.width * 1.6), int(ros_image.height * 1.6))) # Make a copy of the screen as the result, and draw the result on it
        rgb_image = cv2.resize(rgb_image, (int(ros_image.width / 2), int(ros_image.height / 2))) 
        results = self.pose.process(rgb_image)  # recognize
        self.drawing.draw_landmarks(result_image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS) # Draw the joints and connections
        self.fps.update() # Calculate frame rate
        result_image = self.fps.show_fps(result_image) # Display the frame rate on the result screen
        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        pose_node = PoseNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
