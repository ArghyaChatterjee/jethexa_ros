#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps

mp_drawing = mp.solutions.drawing_utils
mp_objectron = mp.solutions.objectron


class ObjectronNode:
    def __init__(self):
        rospy.init_node("objectron_node")
        self.objectron = mp_objectron.Objectron(
                static_image_mode=False,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.8,
                model_name='Cup' # {'Shoe', 'Chair', 'Cup', 'Camera'}
        )
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        self.fps = fps.FPS()

    def image_callback(self, ros_image):
        rospy.loginfo('Received an image! ')
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # raw RGB image
        org_h, org_w = rgb_image.shape[:2]
        result_image = np.copy(cv2.resize(rgb_image, (int(org_w * 1.6), int(org_h * 1.6)))) # Make a copy for display of results in case the image was modified during processing
        rgb_image = cv2.resize(rgb_image, (int(org_w / 4), int(org_h/4)), cv2.INTER_NEAREST) # scale the picture
        try:
            results = self.objectron.process(rgb_image)
            if results.detected_objects: #  the object is recognized
                for detected_object in results.detected_objects:
                    mp_drawing.draw_landmarks(result_image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
                    mp_drawing.draw_axis(result_image, detected_object.rotation, detected_object.translation)
        except Exception as e:
            rospy.logerr(str(e))

        #self.fps.update()
        #result_image = self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('image', result_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        objectron_node = ObjectronNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
