#!/usr/bin/env python3
import enum
import threading
import math
import cv2
import time
import numpy as np
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from vision_utils import fps, vector_2d_angle

from PIL import ImageDraw, ImageFont
from PIL import Image as PIL_Image
import Adafruit_SSD1306


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


def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度
    :param landmarks: 手部关键点
    :return: 各个手指的角度
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list


def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势
    :param angle_list: 各个手指弯曲的角度
    :return : 手势名称字符串
    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.

    gesture_str = "none"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        "none"
    return gesture_str


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    # points = np.expand_dims(np.array(points).astype(dtype=np.int), axis=0)
    points = np.array(points).astype(dtype=np.int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


def get_track_img(points):
    points = np.array(points).astype(dtype=np.int)
    x_min, y_min = np.min(points, axis=0).tolist()
    x_max, y_max = np.max(points, axis=0).tolist()
    track_img = np.full([y_max - y_min + 100, x_max - x_min + 100, 1], 0, dtype=np.uint8)
    points = points - [x_min, y_min]
    points = points + [50, 50]
    draw_points(track_img, points, 1, (255, 255, 255))
    return track_img


class HandGestureNode:
    def __init__(self):
        self.screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
        self.screen.begin() # start the screen
        self.font = ImageFont.truetype('/home/hiwonder/jethexa/src/jethexa_tutorial/misc/wqy-MicroHei.ttf', 16)
        self.screen.image(PIL_Image.fromarray(np.zeros((32, 128), np.uint8)).convert('1')) # create 0 cache 
        self.screen.display() # Display 0 cache, that is to clear the screen
        self.disp_buf = PIL_Image.new('1', (self.screen.width, self.screen.height)) # Create an Image object for drawing
        self.draw = ImageDraw.Draw(self.disp_buf) # create canvas

        rospy.init_node('hand_gesture')

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            # model_complexity=0,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )
        self.drawing = mp.solutions.drawing_utils
        self.fps = fps.FPS()  # fps calculator
        self.gesture = ""

        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_oled)
        rospy.loginfo("hand gesture node created")

    def update_oled(self, _):
        self.draw.rectangle((0, 0, self.screen.width, self.screen.height), outline=0, fill=0) # Filling a black box is to clear the cache to black
        self.draw.text((5, 7), self.gesture.upper(), font=self.font, fill=255) # write ID on the canvas
        self.screen.image(self.disp_buf) # write our graph to the cache
        self.screen.display()


    def image_callback(self, ros_image: Image):
        # rospy.loginfo('Received an image! ')
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        rgb_image = rgb_image[0:360, 320-200:320+200]
        rgb_image = cv2.flip(rgb_image, 1)
        result_image = np.copy(rgb_image)
        gesture = "none"
        try:
            results = self.hand_detector.process(rgb_image)
            if results is not None and results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks(
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    landmarks = get_hand_landmarks(rgb_image, hand_landmarks.landmark)
                    angle_list = hand_angle(landmarks)
                    gesture = h_gesture(angle_list)
                    print(gesture)
        except Exception as e:
            rospy.logerr(str(e))

        self.gesture = gesture
        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


def main(args=None):
    hand_gesture_node = HandGestureNode()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
