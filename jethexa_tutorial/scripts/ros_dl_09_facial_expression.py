#!/usr/bin/env python3
import gc
import queue
import cv2
import rospy
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from vision_utils import fps, vgg, warp_affine
from utils import show_faces, mp_face_location

EXPRESSIONS = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']


class FacialExpressionNode:
    def __init__(self):
        rospy.init_node('facial_expression_node')

        self.fps = fps.FPS() # Frame rate counter

        # Face detector
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            # model_selection=0,
            min_detection_confidence=0.7,
        )

        # Emotion classifier
        fer_model = rospy.get_param("~fer_model", "/home/hiwonder/weights/fer_vgg19_48.trt")
        self.fer = vgg.TrtVGG(fer_model, 48)

        # Subscribe to the camera image thread
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
    
    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        self.image_queue.put(ros_image, block=True) # Push image into queue
        # Because pycuda requires that the context is created and executed in the same thread, if the recognition cannot be performed in the topic callback, it must be placed in the queue to perform the recognition on the main thread

    def image_process(self):
        ros_image = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # raw RGB image

        rgb_image = cv2.resize(rgb_image, (360, 180)) # scale the image
        rgb_image = cv2.flip(rgb_image, 1)
        result_image = np.copy(rgb_image)
        results = self.face_detector.process(rgb_image) # recognize the human face in the image and the face key point

        boxes, keypoints = mp_face_location(results, rgb_image) # Get the output data of face recognition and convert the normalized data to pixel coordinates

        # traverse all the recognized human faces, and recognize their emotion
        for i, (box, landmark) in enumerate(zip(boxes, keypoints)):
            x1, y1, x2, y2 = np.array(box).astype(dtype=np.int)[:4] # Get the coordinates of the face frame
            face = rgb_image[y1:y2, x1:x2] # Cut out the face from the screen
            face = cv2.cvtColor(face, cv2.COLOR_RGB2GRAY)
            face = cv2.cvtColor(face, cv2.COLOR_GRAY2RGB)
            face = warp_affine(face, landmark) # align the human face to improve the accuracy of the emotion classification
            cv2.imshow('face', face)
            rospy.sleep(0.01)
            output = self.fer.execute(face) # classify the human emotion

            # print the probability of each emotion of this face
            for j, i in enumerate(output): 
                print(EXPRESSIONS[j] + ":{:0.2f}".format(i))
            print("")

            # Write the name of the recognized expression on the upper left corner of the face in the screen
            idx = np.argmax(output) # Find the expression subscript with the highest probability
            s = EXPRESSIONS[idx] + ' {:0.2f}'.format(output[idx])
            cv2.putText(result_image, s, (x1 + 5, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        result_image = show_faces(rgb_image, result_image, boxes, keypoints) # Display the recognized face and face key points on the screen

        self.fps.update() # Update fps stats
        result_image = self.fps.show_fps(result_image) # fps is displayed on the screen
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR) 

        cv2.imshow('image', result_image)
        cv2.waitKey(1)
        gc.collect()


def main():
    try:
        facial_expression_node = FacialExpressionNode()
        while not rospy.is_shutdown():
            facial_expression_node.image_process()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == "__main__":
    main()

