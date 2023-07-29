#!/usr/bin/python3

import math
import threading
import cv2
import rospy
import numpy as np
from apriltag import apriltag
from sensor_msgs.msg import CameraInfo, Image
from vision_utils import fps, point_remapped
from PIL import ImageDraw, ImageFont
from PIL import Image as PIL_Image
import Adafruit_SSD1306

OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

AXIS = np.float32([[0, 0, 0],
                   [1.5, 0, 0], 
                   [0, 1.5, 0], 
                   [0, 0, 1.5]])

CIRCLE = np.float32([[0.3 * math.cos(math.radians(i)), 0.3 * math.sin(math.radians(i)), 0] for i in range(360)])
AXIS = np.append(AXIS, CIRCLE, axis=0)


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    cv2.drawContours(img, [imgpts[4:]],-1,(255, 255, 0), -1)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]),(255, 0, 0),3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]),(0, 255, 0),3)
    cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]),(0, 0, 255),3)
    return img


class TagNode:
    def __init__(self, name):
        self.screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
        self.screen.begin() # start the screen
        self.font = ImageFont.truetype('/home/hiwonder/jethexa/src/jethexa_tutorial/misc/wqy-MicroHei.ttf', 14)
        self.screen.image(PIL_Image.fromarray(np.zeros((32, 128), np.uint8)).convert('1')) # create 0 cache
        self.screen.display() # display 0 cache that is clear the image
        self.disp_buf = PIL_Image.new('1', (self.screen.width, self.screen.height)) # Create an image object for drawing
        self.draw = ImageDraw.Draw(self.disp_buf) # create canvas

        rospy.init_node(name)
        self.camera_intrinsic = np.matrix([[619.063979, 0,          302.560920],
                                           [0,          613.745352, 237.714934],
                                           [0,          0,          1]])

        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        self.tag_detector = apriltag("tag36h11")
        self.fps = fps.FPS()
        self.lock = threading.Lock()

        # Subscribe to the camera image topic, and subscribe to the camera parameter topic
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(self.camera_rgb_prefix + '/camera_info', CameraInfo, self.camera_info_callback)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_oled)
        self.tag = 0, (0, 0)
    
    def update_oled(self, _):
        string = []
        with self.lock:
            if self.tag is not None:
                string.append("TAG ID:{:>3d}".format(self.tag[0]))
                string.append("X:{:.2f}".format(self.tag[1][0]))
                string.append("Y:{:.2f}".format(self.tag[1][1]))

        if string:
            self.draw.rectangle((0, 0, self.screen.width, self.screen.height), outline=0, fill=0) # Filling a black box is to clear the cache to black
            self.draw.text((1, 0), string[0], font=self.font, fill=255) # write ID on the canvas
            self.draw.text((1, 16), string[1], font=self.font, fill=255) # draw the position on canvas
            self.draw.text((62, 16), string[2], font=self.font, fill=255) # draw the position on canvas
            self.screen.image(self.disp_buf) # write our graph to the cache
            self.screen.display()

    def camera_info_callback(self, msg):
        # rospy.loginfo("image info recv")
        with self.lock:
            self.camera_intrinsic = np.matrix(msg.K).reshape(1, -1, 3)
            self.dist_coeffs = np.array(msg.D)

    def image_callback(self, ros_image: Image):
        # rospy.loginfo("image recv")
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # Convert custom image messages to images
        result_image = np.copy(rgb_image)
        try:
            with self.lock:
                result_image = self.image_proc(rgb_image, result_image)
        except Exception as e:
                rospy.logerr(str(e))
        self.fps.update()
        self.fps.show_fps(result_image)
        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        
    def image_proc(self, rgb_image, result_image):
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY) # Convert RGB image to grayscale
        gray = cv2.resize(gray, (320, 180)) 
        detections = self.tag_detector.detect(gray)
        if detections:
            for detection in detections: # Traverse all recognized tags
                tag_id = detection['id']
                tag_center = detection['center']
                tag_corners = detection['lb-rb-rt-lt']
                corners =[point_remapped(p, (320, 180), (640, 360)) for p in tag_corners]
                lb, rb, rt, lt = corners
                cv2.circle(result_image, (int(lb[0]), int(lb[1])), 2, (0, 255, 255), -1)
                cv2.circle(result_image, (int(lt[0]), int(lt[1])), 2, (0, 255, 255), -1)
                cv2.circle(result_image, (int(rb[0]), int(rb[1])), 2, (0, 255, 255), -1)
                cv2.circle(result_image, (int(rt[0]), int(rt[1])), 2, (0, 255, 255), -1)
                # cv2.circle(result_image, (int(tag_center[0]), int(tag_center[1])), 3, (255, 0, 0), -1)
                tag_center = point_remapped(tag_center,(320,180), (640, 360))
                corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
                ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_intrinsic, self.dist_coeffs)
                imgpts, jac = cv2.projectPoints(AXIS, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                result_image = draw(result_image, corners, imgpts)

                self.tag = (tag_id, tag_center)
        else:
            self.tag = 0, (0, 0)
        return result_image


def main():
    try:
        tag_node = TagNode('tag_disp')
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == '__main__':
    main()
