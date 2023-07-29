#!/usr/bin/env python3
import os
import sys
import cv2
import time
import rospy
import queue
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import yolov5, fps


TRT_INPUT_SIZE = 160
TRT_NUM_CLASSES = 2
FACEMASK_LABELS = ("nomask", "mask")
COLORS = ((255, 0, 0), (0, 0, 255))


class FacemaskNode:
    def __init__(self):
        rospy.init_node('facemask_node')

        # Create a Yolo instance
        weights = rospy.get_param("~weights", "/home/hiwonder/weights/facemask_v5_160.trt") # Get parameters and weight file path
        print(weights)
        self.yolov5 = yolov5.TrtYolov5(weights, TRT_INPUT_SIZE, TRT_NUM_CLASSES)

        self.fps = fps.FPS() # Frame rate counter

        # Subscribe to the camera image thread
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
    
    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        self.image_queue.put(ros_image, block=True) # push image into queue
        # Because pycuda requires that the context is created and executed in the same thread, if the recognition cannot be performed in the topic callback, it must be placed in the queue to perform the recognition on the main thread

    def image_process(self):
        ros_image = self.image_queue.get(block=True) # Remove the screen from the queue

        # Convert the screen to opencv format
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)

        try:
            outputs = self.yolov5.detect(rgb_image) # recognize the image
            # Post-processing. Convert raw output to bounding boxes for NMS thresholding, etc.
            boxes, confs, classes = self.yolov5.post_process(rgb_image, outputs, 0.6, 0.2) 
            height, width = rgb_image.shape[:2]

            for box, cls_id, cls_conf in zip(boxes, classes, confs):
                x1 = box[0] / TRT_INPUT_SIZE * width
                y1 = box[1] / TRT_INPUT_SIZE * height
                x2 = box[2] / TRT_INPUT_SIZE * width
                y2 = box[3] / TRT_INPUT_SIZE * height


                # The result screen shows whether you wear a mask or not.
                cv2.putText(result_image, 
                            FACEMASK_LABELS[cls_id] + " " + str(float(cls_conf))[:4],
                            (int(x1), int(y1) - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[cls_id], 2)
                # The mask is framed in the result screen
                cv2.rectangle(result_image, 
                              (int(x1), int(y1)), (int(x2), int(y2)), 
                              COLORS[cls_id], 3)

                rospy.loginfo((cls_id, float(cls_conf), x1, x2, y1, y2))
        except Exception as e:
            rospy.logerr(str(e))
        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("image", result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        facemask_node = FacemaskNode()
        while not rospy.is_shutdown():
            facemask_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))
