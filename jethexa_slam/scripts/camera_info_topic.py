#!/usr/bin/env python3

import sys
import rospy
import yaml
from sensor_msgs.msg import CameraInfo


class CameraInfoNode:
    def __init__(self):
        rospy.init_node("camera_info_topic", anonymous=True, log_level=rospy.INFO)
        self.publisher = rospy.Publisher("/rgb_camera_info", CameraInfo, queue_size=1)
        self.count = 0

        self.info = None
        with open('/home/hiwonder/.ros/camera_info/camera.yaml', 'r') as f:
            b = f.read()
            self.info = yaml.safe_load(b)

        if self.info is None:
            rospy.logerr("Can not load camera.yaml")
            sys.exit(-1)

        rospy.Timer(rospy.Duration(1.0 / 30), self.callback)

    def callback(self, event):

        self.count += 1
        msg = CameraInfo()
        msg.header.seq = self.count
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "jethexa/camera_rgb_optical_frame"
        msg.height = self.info['image_height']
        msg.width = self.info['image_width']
        msg.distortion_model = self.info['distortion_model']
        msg.D = self.info['distortion_coefficients']['data']
        msg.K = self.info['camera_matrix']['data']
        msg.R = self.info['rectification_matrix']['data']
        msg.P = self.info['projection_matrix']['data']
        self.publisher.publish(msg)





if __name__ == "__main__":
    node = CameraInfoNode() # create related resources
    rospy.spin()

