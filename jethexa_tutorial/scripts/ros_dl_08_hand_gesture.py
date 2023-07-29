#!/usr/bin/env python3
import enum
import rospy
import cv2
import time
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps, distance, vector_2d_angle
from jethexa_sdk import buzzer


def get_hand_landmarks(img_size, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    w, h = img_size
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度
    :param landmarks: 手部关键点
    :return: 各个手指的角度
    """
    angle_list = []
    # thumb 
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index finger
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle finger
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring finger
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pinkie
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
    thr_angle, thr_angle_thumb, thr_angle_s = 65.0, 53.0, 49.0
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "gun"
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
        gesture_str = "none"
    return gesture_str


class State(enum.Enum):
    NULL = 0
    TRACKING = 1


def draw_points(img, points, tickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, tickness)


class HandGestureNode:
    def __init__(self):
        rospy.init_node("hand_gesture_node_")
        self.drawing = mp.solutions.drawing_utils
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            #model_complexity=0,
            min_tracking_confidence=0.2,
            min_detection_confidence=0.7
        )

        self.fps = fps.FPS()  # fps calculator
        self.state = State.NULL
        self.points = [[0, 0], ]
        self.no_finger_timestamp = time.time()
        self.one_count = 0
        self.direction = ""

        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

        rospy.loginfo("hand gesture node created")


    def image_callback(self, ros_image):
        #rospy.loginfo('Received an image! ')
        # Convert ros format image to opencv format
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # raw RGB image
        rgb_image = cv2.flip(rgb_image, 1) # mirror image
        result_image = np.copy(rgb_image) # Make a copy for display of results in case the image was modified during processing

        if time.time() - self.no_finger_timestamp > 2:
            self.direction = ""
        else:
            if self.direction != "":
                cv2.putText(result_image, self.direction, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        try:
            results = self.hand_detector.process(rgb_image) # recognize hand and hand key points
            if results.multi_hand_landmarks:
                gesture = "none"
                index_finger_tip = [0, 0]
                for hand_landmarks in results.multi_hand_landmarks: 
                    self.drawing.draw_landmarks(
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    h, w = rgb_image.shape[:2]
                    landmarks = get_hand_landmarks((w, h), hand_landmarks.landmark)
                    angle_list = hand_angle(landmarks)
                    gesture = h_gesture(angle_list) # judge the gesture based on the hand key point position
                    index_finger_tip = landmarks[8].tolist()

                cv2.putText(result_image, gesture, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 5)
                cv2.putText(result_image, gesture, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

                if self.state == State.NULL:
                    if gesture == "one":  # Detect the open palm, and start recognizing the index finger tip
                        self.other_count = 0
                        self.one_count += 1
                        if self.one_count > 30:
                            self.one_count = 0
                            self.state = State.TRACKING
                            buzzer.beep(0.05, 2, rospy.sleep)
                    else:
                        self.one_count = 0

                elif self.state == State.TRACKING:
                    if gesture == "one":
                        self.no_finger_timestamp = time.time()
                    if distance(index_finger_tip, self.points[-1]) > 10:
                        self.points.append(index_finger_tip)
                        self.one_count  = 0
                    else:
                        self.one_count += 1

                    if self.one_count > 40:
                        buzzer.beep(0.1, 1, rospy.sleep)

                        # Calculate the angle between the line connecting the end point and the start point of the trajectory relative to the horizontal vector
                        v1 = np.array([0, self.points[1][1]])
                        v2 = np.array([self.points[-1][0] - self.points[1][0], 
                                       self.points[-1][1] - self.points[1][1]])
                        angle =vector_2d_angle(v1, v2)
                        print(angle)

                        if abs(angle - 90) < 20:
                            self.direction = "RIGHT TO LEFT"
                        elif abs(angle - -90) < 20:
                            self.direction = "LEFT TO RIGHT"
                        elif abs(angle - 0) < 20:
                            self.direction = "UP TO DOWN"
                        elif abs(angle - 180) < 20:
                            self.direction = "DOWN TO UP"
                        else:
                            self.direction = ""

                        self.state = State.NULL
                        self.one_count = 0
                        self.points = [[0, 0],]
                else:
                    pass
                draw_points(result_image, self.points[1:])
            else:
                if self.state != State.NULL:
                    if time.time() - self.no_finger_timestamp > 2:
                        self.one_count = 0
                        self.points = [[0, 0],]
                        self.state = State.NULL
                        buzzer.beep(0.5, 1, rospy.sleep)
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        hand_gesture_node = HandGestureNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        
