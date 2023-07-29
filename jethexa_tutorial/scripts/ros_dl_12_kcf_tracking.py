#!/usr/bin/env python3
import cv2
import rospy
import queue
import numpy as np
from sensor_msgs.msg import Image
from vision_utils import fps
from jethexa_controller import client
from jethexa_sdk import pid


class KCFTrackingNode:
    def __init__(self):
        rospy.init_node('kcf_node')

        # Instantiate the kcf tracker
        self.tracker = None
        self.enable_select = False
        self.fps = fps.FPS() # Frame rate counter
        #self.jethexa = client.Client(self)
        #self.pid = pid.PID(0.1, 0, 0)

        # Subscribe to the camera image thread
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)


    def image_callback(self, ros_image):
        #rospy.logdebug('Received an image! ')
        # Convert the screen to opencv format
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(cv2.resize(rgb_image, (int(ros_image.width * 1.6), int(ros_image.height * 1.6))))
        rgb_image = cv2.resize(rgb_image, (int(ros_image.width / 4), int(ros_image.height / 4)))
        factor = result_image.shape[0] / rgb_image.shape[0]

        try:
            if self.tracker is None:
                if self.enable_select:
                    roi = cv2.selectROI("image", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), False)
                    roi =  tuple(int(i / factor)for i in roi)
                    if roi:
                        self.tracker = cv2.TrackerCSRT_create()
                        #self.tracker = cv2.TrackerKCF_create()
                        self.tracker.init(rgb_image, roi)
            else:
                status, box = self.tracker.update(rgb_image)
                if status:
                    rospy.loginfo(str(box))
                    p1 = int(box[0] * factor), int(box[1] * factor)
                    p2 = p1[0] + int(box[2] * factor), p1[1] + int(box[3] * factor)
                    cv2.rectangle(result_image, p1, p2, (255, 255, 0), 2)

        except Exception as e:
            rospy.logerr(str(e))

        self.fps.update()
        self.fps.show_fps(result_image)
        result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
        cv2.imshow("image", result_image)

        key = cv2.waitKey(1)
        if key == ord('s'): # Press s to start selecting the tracking target
            self.tracker = None
            self.enable_select = True


if __name__ == '__main__':
    try:
        kcf_tracking = KCFTrackingNode()
        print("在画面窗口按下s开始选择追踪目标")
        print("在画面窗口按下s开始选择追踪目标")
        print("在画面窗口按下s开始选择追踪目标")
        print("在画面窗口按下s开始选择追踪目标")
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
