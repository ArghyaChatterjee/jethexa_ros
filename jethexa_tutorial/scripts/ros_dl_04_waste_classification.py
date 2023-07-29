#!/usr/bin/env python3

import cv2
import rospy
import queue
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import yolov5, fps


TRT_INPUT_SIZE = 160
TRT_NUM_CLASSES = 12
TRT_CLASS_NAMES = ('Banana Peel', 'Broken Bones', 'Cigarette End', 'Disposable Chopsticks',
                   'Ketchup', 'Marker', 'Oral Liquid Bottle', 'Plate',
                   'Plastic Bottle', 'Storage Battery', 'Toothbrush', 'Umbrella')

WASTE_CLASSES = {
    'food_waste': ('Banana Peel', 'Broken Bones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'Oral Liquid Bottle', 'Storage Battery'),
    'recyclable_waste': ('Plastic Bottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'Cigarette End', 'Disposable Chopsticks'),
}

COLORS = {
    'recyclable_waste': (0, 0, 255),
    'hazardous_waste': (255, 0, 0),
    'food_waste': (0, 255, 0),
    'residual_waste': (80, 80, 80)
}

class WasteClassificationNode:
    def __init__(self):
        rospy.init_node('waste_classification_node')

        # Create a Yolo instance
        weights = rospy.get_param("~weights", "/home/hiwonder/waste_v5_160.trt") # Get parameters and weight file path
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
            # Post-processing. Convert raw output to bounding boxes for NMS thresholding.
            boxes, confs, classes = self.yolov5.post_process(rgb_image, outputs, 0.65) 
            width = rgb_image.shape[1]
            height = rgb_image.shape[0]
            cards = []

            for box, cls_conf, cls_id in zip(boxes, confs, classes):
                x1 = int(box[0] / TRT_INPUT_SIZE * width)
                y1 = int(box[1] / TRT_INPUT_SIZE * height)
                x2 = int(box[2] / TRT_INPUT_SIZE * width)
                y2 = int(box[3] / TRT_INPUT_SIZE * height)
                waste_name = TRT_CLASS_NAMES[cls_id]
                waste_class_name = ''
                for k, v in WASTE_CLASSES.items():
                    if waste_name in v:
                        waste_class_name = k
                        break
                cards.append((cls_conf, x1, y1, x2, y2, waste_class_name))
                result_image = cv2.putText(result_image, waste_name + " " + str(float(cls_conf))[:4], (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS[waste_class_name], 2)
                result_image = cv2.rectangle(result_image, (x1, y1), (x2, y2), COLORS[waste_class_name], 3)
        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update() # Update fps stats
        result_image = self.fps.show_fps(result_image) # fps is displayed on the screen
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('result', result_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        waste_classification_node = WasteClassificationNode()
        while not rospy.is_shutdown():
            waste_classification_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))
