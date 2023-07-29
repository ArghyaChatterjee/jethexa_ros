from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
import rospy
import numpy as np
from sensor_msgs.msg import Image


class VideoCapture(QObject):
    new_img_signal = pyqtSignal(QImage)

    def __init__(self, path):
        super(VideoCapture, self).__init__()
        self.path = path
        self.image_sub = rospy.Subscriber(self.path, Image, self.image_callback)

    def __del__(self):
        self.image_sub.unregister()

    def image_callback(self, ros_image):
        frame = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        img = QImage(frame.data, ros_image.width, ros_image.height, QImage.Format_RGB888).scaled(320, 240)
        self.new_img_signal.emit(img)
