#!/usr/bin/env python3
import gc
import queue
import cv2
import time
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
        self.screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
        self.screen.begin() # start the screen
        self.font = ImageFont.truetype('/home/hiwonder/jethexa/src/jethexa_tutorial/misc/wqy-MicroHei.ttf', 16)
        self.screen.image(PIL_Image.fromarray(np.zeros((32, 128), np.uint8)).convert('1')) # create 0 cache
        self.screen.display() # Display 0 cache, that is to clear the screen
        self.disp_buf = PIL_Image.new('1', (self.screen.width, self.screen.height)) # Create an Image object for drawing
        self.draw = ImageDraw.Draw(self.disp_buf) # create canvas

        rospy.init_node(name, log_level=log_level)
        self.node_name = name

        self.fps = fps.FPS() # Frame rate counter

        # face detector
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            # model_selection=0,
            min_detection_confidence=0.7,
        )

        # emotion classifier
        fer_model = rospy.get_param("~fer_model", "/home/hiwonder/weights/fer_vgg19_48.trt")
        self.fer = vgg.TrtVGG(fer_model, 48)

        self.expression = '', ''
        # Subscribe to the camera image thread
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        self.timestamp = time.time()
        self.time_count = 0
    
    def update_oled(self):
        self.draw.rectangle((0, 0, self.screen.width, self.screen.height), outline=0, fill=0) # Filling a black box is to clear the cache to black
        self.draw.text((1, 2), self.expression[0] + ' ' + self.expression[1], font=self.font, fill=255) # write ID on the canvas
        self.screen.image(self.disp_buf) # write our graph to the cache
        try:
            self.screen.display()
        except Exception as e:
            pass
    
    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        try:
            self.image_queue.put_nowait(ros_image)
        except Exception as e:
            pass

    def image_process(self):
        ros_image = self.image_queue.get(block=True)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        result_image = np.copy(rgb_image)
        o_h, o_w = rgb_image.shape[:2]
        rgb_image = cv2.resize(rgb_image, (int(o_w / 4), int(o_h / 4))) # scale the image
        results = self.face_detector.process(rgb_image) # Recognize faces and face key points in images
        boxes, keypoints = mp_face_location(results, result_image) # Get the output data of face recognition and convert the normalized data to pixel coordinates

        expression = "", ""
        # Traverse all recognized faces and perform emotion recognition on these faces
        for i, (box, landmark) in enumerate(zip(boxes, keypoints)):
            x1, y1, x2, y2 = np.array(box).astype(dtype=np.int)[:4] # Get the coordinates of the face frame
            face = result_image[y1:y2, x1:x2] # Cut out the face from the screen
            face = warp_affine(face, landmark) # align face to improve the accuracy of emotion classification
            face = cv2.cvtColor(face, cv2.COLOR_RGB2GRAY)
            face = cv2.cvtColor(face, cv2.COLOR_GRAY2RGB)
            # cv2.imshow('face', face)
            output = self.fer.execute(face) # Emotion classification for human

            # Print probability of each emotion of this face
            for j, i in enumerate(output): 
                print(EXPRESSIONS[j] + ":{:0.2f}".format(i))
            print("")

            # Write the name of the recognized emotion on the upper left corner of the face in the screen
            idx = np.argmax(output) # Find the expression subscript with the highest probability
            s = EXPRESSIONS[idx] + ' {:0.2f}'.format(output[idx])
            cv2.putText(result_image, s, (x1 + 5, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            expression = EXPRESSIONS[idx], "{:0.2f}".format(output[idx])

        self.expression = expression

        result_image = show_faces(result_image, result_image, boxes, keypoints) # Display the recognized face and face key points on the screen

        self.fps.update() # Update fps stats
        result_image = self.fps.show_fps(result_image) # fps is displayed on the screen
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR) 

        cv2.imshow('image', result_image)
        cv2.waitKey(1)
        if time.time() - self.timestamp > 0.5:
            self.timestamp = time.time()
            self.update_oled()
            self.time_count += 1
            self.time_count = 0
            gc.collect() #Not manual gc will crash. Better automatic gc strategy can be set to improve performance


def main():
        facial_expression_node = FacialExpressionNode('facial_expression_node', log_level=rospy.INFO)
        while not rospy.is_shutdown():
            try:
                facial_expression_node.image_process()
            except Exception as e:
                rospy.loger(str(e))

if __name__ == "__main__":
    main()

