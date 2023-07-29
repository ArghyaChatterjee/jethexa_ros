#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps
import gc


class FaceDetectNode:
    def __init__(self):
        rospy.init_node("face_detect_node")
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            # model_selection=0,
            min_detection_confidence=0.6,
        )
        self.drawing = mp.solutions.drawing_utils

        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        self.fps = fps.FPS()

    def image_callback(self, ros_image):
        # self.get_logger().debug('Received an image! ')
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # raw RGB image
        result_image = np.copy(rgb_image) # Make a copy for display of results in case the image was modified during processing
        rgb_image = cv2.resize(rgb_image, (320, 180), cv2.INTER_NEAREST) # scale the picture
        
        try:
            results = self.face_detector.process(rgb_image) # call face detection
            if results.detections: # face is recognized
                for detection in results.detections: # Frame the face on the screen
                    self.drawing.draw_detection(result_image, detection)
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update()
        result_image = self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)
        gc.collect()


if __name__ == "__main__":
    try:
        face_detection_node = FaceDetectNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
