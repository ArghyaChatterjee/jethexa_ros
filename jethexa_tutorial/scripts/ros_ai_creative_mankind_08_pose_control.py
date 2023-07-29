#!/usr/bin/env python3
import cv2
import math
import enum
import time
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from jethexa_controller import jethexa
from jethexa_sdk import buzzer, serial_servo
from vision_utils import fps, distance, vector_2d_angle

# The position value corresponding to each 1 degree rotation of the servo
PULSE_PER_DEGREE = 1000 / 240

# The position value of the servo corresponding to the initial position of each servo
RIGHT_SERVO_I_DEFAULT = 340
RIGHT_SERVO_M_DEFAULT = 425
RIGHT_SERVO_O_DEFAULT = 235
LEFT_SERVO_I_DEFAULT = 660
LEFT_SERVO_M_DEFAULT = 575
LEFT_SERVO_O_DEFAULT = 775


class State(enum.Enum):
    NULL = 0
    INTO_IMITATION_1 = 1
    IMITATION = 2


def get_angle(p1, p2, p3):
    """
    获取三个点形成的夹角
    :param p1: 第一个点
    :param p2: 第二个点
    :param p3: 第三个点
    :return: 角度
    """
    angle = vector_2d_angle(p2 - p1, p3 - p2)
    return angle


def is_pentagon(landmarks):
    """
    通过手工2d几何特征判断两手是否举过头顶, 并且手臂与肩部形成五边形
    :param landmarks: 肢体的各个关键点
    :return: True or False 符合或不符合
    """
    shoulder_width = distance(landmarks[12], landmarks[11])  # shoulder width
    hand_dist = distance(landmarks[16], landmarks[15])  #distance between wrists
    if hand_dist > shoulder_width:  # Wrist distance is less than shoulder width
        return False
    for p in landmarks[:7]:  # Both wrists should be raised above the head, higher than the eyes and nose
        if landmarks[15][1] > p[1] or landmarks[16][1] > p[1]:
            return False
    if get_angle(landmarks[11], landmarks[12], landmarks[14]) < 40:
        return False  # The left upper arm should be raised more than 40 degrees above the shoulder
    if get_angle(landmarks[13], landmarks[11], landmarks[12]) < 40:
        return False  # The right upper arm should be raised more than 40 degrees above the shoulder
    return True


def is_level(landmarks, angle_threshold=15):
    """
    通过手工2d几何特征判断肩部是否和画面水平
    :param landmarks:
    :param angle_threshold:
    :return:
    """
    p0 = landmarks[12].copy()
    p0[0] = 0
    if abs(get_angle(p0, landmarks[12], landmarks[11])) > angle_threshold:
        return False
    return True


def is_flat(landmarks, angle_threshold):
    """
    通过手工2d几何特征判断双臂是否展开
    :param landmarks:
    :param angle_threshold:
    :return: True or False 符合或不符合
    """
    arm_marks = [15, 13, 11, 12, 14, 16]
    for i in range(3):
        angle = get_angle(landmarks[arm_marks[i]], landmarks[arm_marks[i + 1]], landmarks[arm_marks[i + 2]])
        if abs(angle) > angle_threshold:
            return False
    return is_level(landmarks, angle_threshold)


def is_cross(landmarks):
    """
    通过手工2d几何特征判断双臂是否居高并交叉
    :param landmarks:
    :return: True of False
    """
    if landmarks[16][0] <= landmarks[15][0]:
        return False
    return is_pentagon(landmarks)


def mp_pose_landmarks(results, img_rgb, draw=True):
    lm_list = []
    if results and results.pose_landmarks:
        h, w, = img_rgb.shape[:2]
        for idx, lm in enumerate(results.pose_landmarks.landmark):
            cx, cy = int(lm.x * w), int(lm.y * h)
            lm_list.append([cx, cy])
            if draw:
                cv2.circle(img_rgb, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        cv2.circle(img_rgb, lm_list[11], 5, (255, 255, 0), cv2.FILLED)
        cv2.circle(img_rgb, lm_list[12], 5, (0, 255, 255), cv2.FILLED)
        cv2.circle(img_rgb, lm_list[14], 5, (0, 255, 0), cv2.FILLED)

    if len(lm_list) > 0:
        return np.array(lm_list)
    else:
        return None


class MankindPoseNode:
    def __init__(self):
        rospy.init_node("pose_control_node")
        self.state = State.NULL
        self.timestamp = 0
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.4
        )
        self.drawing = mp.solutions.drawing_utils
        self.controller = serial_servo
        self.jethexa = jethexa.JetHexa(self)
        self.jethexa.run_action_set("/home/hiwonder/ActionSets/body_1.d6a", 1)
        rospy.sleep(1)
        self.r_x_dist = 0
        self.l_x_dist = 0
        self.count = 0

        self.fps = fps.FPS()
        self.fps.fps = 10
        self.fps.update()
        self.reset_servos()

        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

        rospy.loginfo("human pose node created")

    def reset_servos(self):
        self.controller.set_position(5, RIGHT_SERVO_I_DEFAULT, 1000)
        self.controller.set_position(3, RIGHT_SERVO_M_DEFAULT, 1000)
        self.controller.set_position(1, RIGHT_SERVO_O_DEFAULT, 1000)
        self.controller.set_position(6, LEFT_SERVO_I_DEFAULT, 1000)
        self.controller.set_position(4, LEFT_SERVO_M_DEFAULT, 1000)
        self.controller.set_position(2, LEFT_SERVO_O_DEFAULT, 1000)

    def stop_imitation(self):
        self.state = State.NULL
        self.reset_servos()
        buzzer.beep(0.5, 1, rospy.sleep)

    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        # raw RGB image
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) 
        rgb_image = cv2.flip(rgb_image, 1)  # mirror image
        result_image = np.copy(rgb_image)

        results = self.pose.process(rgb_image) # start recognizing
        if results:
            self.drawing.draw_landmarks(result_image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
            landmarks = mp_pose_landmarks(results, result_image, True)  # process
            dur = int(1.0 / self.fps.fps * 1000) + 20  # control the motion speed according to the frame rate
            if landmarks is not None:
                if self.state == State.NULL:
                    if time.time() - self.timestamp > 5:
                        if is_pentagon(landmarks):  # Raise your hands high to start the test and enter the imitation mode
                            self.state = State.INTO_IMITATION_1
                            self.timestamp = time.time()
                            buzzer.beep(0.1, 2, rospy.sleep)

                elif self.state == State.INTO_IMITATION_1:
                    if is_flat(landmarks, 30):  # Extend hands horizontally
                        self.count += 1
                        if self.count > 2:
                            self.state = State.IMITATION  # Include imitation mode
                            # Calculate the pixel length of the two upper arms
                            self.r_x_dist = distance(landmarks[13], landmarks[11])
                            self.l_x_dist = distance(landmarks[12], landmarks[14])
                            buzzer.beep(0.1, 2, rospy.sleep)
                    else:
                        self.count = 0
                        # Timeout. Re-recognize without successfully entering imitation mode
                        if time.time() - self.timestamp > 5:
                            self.state = State.NULL
                            buzzer.beep(0.5, 1, rospy.sleep)
                else:
                    self.timestamp = time.time()
                    # imitate the action
                    if not is_cross(landmarks):
                        left_angle_2 = get_angle(landmarks[11], landmarks[12], landmarks[14])
                        left_angle_3 = get_angle(landmarks[12], landmarks[14], landmarks[16])
                        right_angle_2 = get_angle(landmarks[12], landmarks[11], landmarks[13])
                        right_angle_3 = get_angle(landmarks[11], landmarks[13], landmarks[15])
                        right_servo_2 = min(max(int(RIGHT_SERVO_M_DEFAULT + PULSE_PER_DEGREE * right_angle_2), 100), 800)
                        right_servo_3 = min(max(int(RIGHT_SERVO_O_DEFAULT + PULSE_PER_DEGREE * right_angle_3), 80), 850)
                        left_servo_2 = min(max(int(LEFT_SERVO_M_DEFAULT + PULSE_PER_DEGREE * left_angle_2), 200), 900)
                        left_servo_3 = min(max(int(LEFT_SERVO_O_DEFAULT + PULSE_PER_DEGREE * left_angle_3), 150), 920)

                        r_x_dist = distance(landmarks[13], landmarks[11])
                        l_x_dist = distance(landmarks[12], landmarks[14])
                        left_angle_1 = min(max(90.0 - l_x_dist / self.l_x_dist * 90.0, 0), 120)
                        right_angle_1 = min(max(90.0 - r_x_dist / self.r_x_dist * 90.0, 0), 120)
                        left_servo_1 = int(LEFT_SERVO_I_DEFAULT - PULSE_PER_DEGREE * left_angle_1)
                        right_servo_1 = int(RIGHT_SERVO_I_DEFAULT + PULSE_PER_DEGREE * right_angle_1)

                        self.controller.set_position(5, right_servo_1, dur)
                        self.controller.set_position(3, right_servo_2, dur)
                        self.controller.set_position(1, right_servo_3, dur)
                        self.controller.set_position(6, left_servo_1, dur)
                        self.controller.set_position(4, left_servo_2, dur)
                        self.controller.set_position(2, left_servo_3, dur)
                    else:  # Raise and cross hands to exit imitation mode 
                        self.stop_imitation()

            else:
                # Timeout. Exit imitation mode without successfully recognizing the body
                if time.time() - self.timestamp > 2 and self.state != State.NULL:
                    self.stop_imitation()

        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        pose_node = MankindPoseNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
