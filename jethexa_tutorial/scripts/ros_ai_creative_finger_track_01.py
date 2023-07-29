#!/usr/bin/env python3
# Draw the trajectory of your index finger on the screen
# Press the space bar to clear the existing trajectory

import cv2
import time
import numpy as np
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from vision_utils import fps


def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


class FingerTrackNode:
    def __init__(self):
        rospy.init_node('finger_track_01')
        self.drawing = mp.solutions.drawing_utils
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,   # recognize one hand at most
            # model_complexity=0,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.fps = fps.FPS()  # fps calculator
        self.points = []
        self.timestamp = time.time()

        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

        rospy.loginfo("finger track node created")

    def image_callback(self, ros_image: Image):
    #    rospy.loginfo('Received an image! ')
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        rgb_image = cv2.flip(rgb_image, 1)
        result_image = np.copy(rgb_image)

        try:
            results = self.hand_detector.process(rgb_image) # start recognizing 
                
            if results is not None and results.multi_hand_landmarks: # the hand key point is recognized
                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks( # draw the hand
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    landmarks = get_hand_landmarks(rgb_image, hand_landmarks.landmark)
                    self.points.append(landmarks[8].tolist()) # save the trajectory of the index finger
                self.timestamp = time.time()
            else:
                if self.timestamp - time.time() > 2: # If the hand is not recognized after a certain period of time, the point list will be cleared.
                    self.points = []

            draw_points(result_image, self.points)

        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('img', result_image)

        key = cv2.waitKey(1)
        if key == ord(' '):
            self.points = []

if __name__ == "__main__":
    try:
        finger_track_node = FingerTrackNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
