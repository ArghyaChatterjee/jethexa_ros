#!/usr/bin/python3

import math
import threading
import rospy
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import sensor_msgs.msg
import Adafruit_SSD1306
from scipy.spatial.transform import Rotation as R


class PostureNode:
    def __init__(self, name):
        self.screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
        self.screen.begin() # start the screen

        # self.font = ImageFont.truetype('/home/hiwonder/jethexa/src/jethexa_tutorial/misc/wqy-MicroHei.ttf', 8)
        self.font = ImageFont.load_default() # load the font

        self.screen.image(Image.fromarray(np.zeros((32, 128), np.uint8)).convert('1')) # create 0 cache
        self.screen.display() # Display 0 cache, that is to clear the screen

        self.disp_buf = Image.new('1', (self.screen.width, self.screen.height)) # Create an image object for drawing
        self.draw = ImageDraw.Draw(self.disp_buf) # build canvas

        rospy.init_node(name)
        self.lock = threading.Lock()

        self.rotate = (0, 0, 0)

        # 订阅 imu 话题
        self.imu_sub = rospy.Subscriber("/imu/filtered", sensor_msgs.msg.Imu, self.imu_callback, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_oled)

    
    def update_oled(self, _):
        string = []
        with self.lock:
            string.append("PITCH:{:>.2f}".format(self.rotate[0]))
            string.append("ROLL:{:.2f}".format(self.rotate[1]))
            string.append("YAW:{:.2f}".format(self.rotate[2]))

        self.draw.rectangle((0, 0, self.screen.width, self.screen.height), outline=0, fill=0) # Filling a black box is to clear the cache to black
        self.draw.text((1, 0), string[0], font=self.font, fill=255) 
        self.draw.text((1, 11), string[1], font=self.font, fill=255) 
        self.draw.text((1, 22), string[2], font=self.font, fill=255)

        # draw the horizontal line
        r = R.from_euler('xyz', (0, 0, self.rotate[0]), degrees=True) # Calculate the coordinates of the horizontal line after rotation using the roll angle
        ((x2, y2, _), ) = r.apply([(25, 0, 0),])
        self.draw.line(((111 - x2, 16 - y2), (111 + x2, 16 + y2)), fill=255, width=3)

        # draw the vertical line
        line_len = (90 - abs(self.rotate[1])) / 90 * 34 / 2  # Calculate the length of the vertical line by the ratio of the angle to 90 degrees
        self.draw.line(((106, 16 - line_len), (106, 16 + line_len)), fill=255, width=2)


        self.screen.image(self.disp_buf) # write our graph to the cache
        self.screen.display()
        
    def imu_callback(self, imu_msg):
        try:
            q = imu_msg.orientation
            r = R.from_quat((q.x, q.y, q.z, q.w))
            self.rotate = r.as_euler('xyz', degrees=True)

        except Exception as e:
            rospy.logerr(str(e))


def main():
    try:
        posture_node = PostureNode('posture_disp')
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == '__main__':
    main()
