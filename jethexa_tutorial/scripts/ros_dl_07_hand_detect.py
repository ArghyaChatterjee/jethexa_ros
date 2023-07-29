#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps


class HandDetectNode:
    def __init__(self):
        rospy.init_node('hand_detect_node')

        # Instantiate a hand recognizer
        self.hand_detector = mp.solutions.hands.Hands( 
            static_image_mode=False,
            max_num_hands=1,
            # model_complexity=0,
            min_tracking_confidence=0.2,
            min_detection_confidence=0.7
        )
        self.drawing = mp.solutions.drawing_utils 

        self.fps = fps.FPS()  # fps calculator

        # Subscribe to camera footage
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)

        rospy.loginfo("hand detect node created")

    def image_callback(self, ros_image: Image):
        # rospy.loginfo('Received an image! ')
        # convert ros image to opencv format
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        rgb_image = cv2.flip(rgb_image, 1) # flip the image for the convenience of viweing, as the it is the mirror image
        result_image = np.copy(rgb_image) # result image
        try:
            results = self.hand_detector.process(rgb_image) # recognize the hand
            if results is not None and results.multi_hand_landmarks: # the hand is recognize
                for hand_landmarks in results.multi_hand_landmarks: # draw the hand
                    self.drawing.draw_landmarks( 
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # Update frame rate counter
        result_image = self.fps.show_fps(result_image) # Display frame rate on screen
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        hand_gesture_node = HandDetectNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
