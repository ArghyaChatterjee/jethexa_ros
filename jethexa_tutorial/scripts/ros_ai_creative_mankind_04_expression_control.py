#!/usr/bin/env python3
import queue
import time
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps, vgg, warp_affine
from utils import show_faces, mp_face_location
from PIL import ImageDraw, ImageFont
from PIL import Image as PIL_Image
import Adafruit_SSD1306

EXPRESSIONS = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']


class FacialExpressionNode:
    def __init__(self, name, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level)
        self.node_name = name

        self.fps = fps.FPS() # Frame rate counter
        self.detect_w, self.detect_h = 640, 360 # Image size for recognition processing

        # face detector
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            # model_selection=0,
            min_detection_confidence=0.7,
        )

        # emotion classifier
        fer_model = rospy.get_param("~fer_model", "/home/hiwonder/weights/fer_vgg19_48.trt")
        self.fer = vgg.TrtVGG(fer_model, 48)
        self.timestamp = time.time()

        self.expression = '', ''
        # subscribe to camera image topic
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
    

    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        self.image_queue.put(ros_image, block=True)

    def image_process(self):
        ros_image = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # raw RGB image
        result_image = np.copy(rgb_image)
        o_h, o_w = rgb_image.shape[:2]
        rgb_image = cv2.resize(rgb_image, (self.detect_w, self.detect_h)) # scale the image
        results = self.face_detector.process(rgb_image) # recognize the human face and face key points in the image

        boxes, keypoints = mp_face_location(results, rgb_image) # Get the output data of face recognition and convert the normalized data to pixel coordinates

        expression = "", ""
        # Traverse all recognized faces and perform emotion recognition on these faces
        for i, (box, landmark) in enumerate(zip(boxes, keypoints)):
            x1, y1, x2, y2 = np.array(box).astype(dtype=np.int)[:4] # Get the coordinates of the face frame
            face = rgb_image[y1:y2, x1:x2] # Cut out the face from the screen
            face = warp_affine(face, landmark) # align the face to improve the accuracy of emotion classification
            cv2.imshow('face', face)
            rospy.sleep(0.01)
            output = self.fer.execute(face) # emotion classification for humans

            # print the probability of each emotion on this face 
            for j, i in enumerate(output): 
                print(EXPRESSIONS[j] + ":{:0.2f}".format(i))
            print("")

            # Write the name of the recognized emotion on the upper left corner of the face in the screen
            idx = np.argmax(output) # Find the expression subscript with the highest probability
            s = EXPRESSIONS[idx] + ' {:0.2f}'.format(output[idx])
            cv2.putText(result_image, s, (x1 + 5, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            expression = EXPRESSIONS[idx], "{:0.2f}".format(output[idx])

        result_image = show_faces(rgb_image, result_image, boxes, keypoints) # Display the recognized face and face key points on the screen

        self.fps.update() # Update fps stats
        result_image = self.fps.show_fps(result_image) # fps is displayed on the screen
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR) 

        if self.expression[0] != expression[0]:
            self.count = 0
            self.expression = expression
        else:
            self.count += 1 

        
        if time.time() > self.timestamp and self.count > 30 and self.expression[0] != "":  
            if self.expression[0] == "Angry":
                self.jethexa.run_action_set("/home/hiwonder/ActionSets/attack.d6a", 1)
                self.timestamp = time.time() + 10
            elif self.expression[1] == "Happy":
                self.jethexa.run_action_set("/home/hiwonder/ActionSets/twist_l.d6a", 1)
                self.timestamp = time.time() + 3
            elif self.gesture == "Neutral":
                self.jethexa.run_action_set("/home/hiwonder/ActionSets/wave.d6a", 1)
                self.timestamp = time.time() + 5
            self.count = 0
            self.expression = "", ""
        

        cv2.imshow('image', result_image)
        cv2.waitKey(1)

def main():
    try:
        facial_expression_node = FacialExpressionNode('facial_expression_node', log_level=rospy.INFO)
        while not rospy.is_shutdown():
            facial_expression_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == "__main__":
    main()

