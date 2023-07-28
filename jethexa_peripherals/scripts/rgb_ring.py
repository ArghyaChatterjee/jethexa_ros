#!/usr/bin/env python3

import rospy
import board
import time
import cv2
import random
import numpy as np
import neopixel_spi as neopixel
import std_msgs.msg


class RGBRing:
    def __init__(self, pixel_num=5):
        self.spi = board.SPI()
        self.pixels_num = pixel_num
        self.pixels = neopixel.NeoPixel_SPI(self.spi, self.pixels_num, pixel_order=neopixel.GRB, auto_write=False)
        self.pixel_index = self.pixels_num - 1
        self.color_index = 0
        self.color_indexs = [30,] * self.pixels_num
        self.saturation = [255, ] * self.pixels_num
        self.values = [255,] * self.pixels_num
        self.values_inc = [5,] * self.pixels_num
        self.breate_rate = [2,]
        self.timestamp = time.time()

    def mode1_update(self):
        self.color_index += 5 # change color
        if self.color_index >= 180: # the top of the color
            self.color_index = self.color_index % 180 # new color value. Modulo
        #  Calculate the color value of the light one by one, and set the bulb color buffer. A circle covers the entire 0-180 range
        for i in range(self.pixels_num):
            # The color interval of each bulb is 180 / the number of bulbs, and the colors between the bulbs will be continuous due to the scattering and mixing of the baffle
            c_index = (self.color_index + int(180 / self.pixels_num) * i) % 180
            # Convert hsv to rgb with opencv
            r, g, b = cv2.cvtColor(np.array([[[c_index, 255, 255], ],], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            self.pixels[i] = int(r << 16 | g << 8 | b) # Combine rgb values into 24bit color values
        self.pixels.show()
        self.timestamp=time.time()

    def mode2_update(self):
        color_inc = [4, 5, 3, 1, 2]
        for i in range(self.pixels_num):
            self.color_indexs[i] += random.randint(1, 6)
            if self.color_indexs[i] > random.randint(175, 180):
                self.color_indexs[i] = 0
        for i in range(self.pixels_num):
            buf = cv2.cvtColor(np.array([[[self.color_indexs[i], 255, 255]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            self.pixels[i] = int(buf[0] << 16 | buf[1] << 8 | buf[2])
        self.pixels.show()


    def mode3_update(self):
        if time.time() - self.timestamp > 0.15:
            colors = 0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0xFF00FF, 0x00FFFF
            for i in range(self.pixels_num):
                self.pixels[i] = colors[random.randint(0, 5)]
            self.timestamp = time.time()
        self.pixels.show()

    def mode4_update(self):
        if self.color_index >= 180:
            self.color_index = 0
        self.color_index += 1
        buf = cv2.cvtColor(np.array([[[self.color_index, 255, 255]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
        data = [int(buf[0] << 16 | buf[1] << 8 | buf[2])] * self.pixels_num
        for i in range(self.pixels_num):
            self.pixels[i] = data[i]
        self.pixels.show()
    

    def mode5_update(self):
        for i in range(self.pixels_num):
            self.values[i] += self.values_inc[i]
            if self.values[i] >= 255:
                self.values[i] = 255
                self.values_inc[i] = -self.values_inc[i]
            if self.values[i] <= 0:
                self.values[i] = 0
                self.values_inc[i] = -self.values_inc[i]
            buf = cv2.cvtColor(np.array([[[self.color_indexs[i], self.saturation[i], self.values[i]]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            self.pixels[i] = int(buf[0] << 16 | buf[1] << 8 | buf[2])
        self.pixels.show()
    
    def mode6_update(self):
        if time.time() - self.timestamp > 0.1:
            for i in range(self.pixels_num):
                buf = cv2.cvtColor(np.array([[[self.color_indexs[i], self.saturation[i], self.values[i]]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
                self.pixels[i] = int(buf[0] << 16 | buf[1] << 8 | buf[2])
            self.pixels.show()
            self.timestamp = time.time()

    def mode0_update(self):
        for i in range(self.pixels_num):
            self.pixels[i] = 0
        self.pixels.show()
    

class RGBRingNode:
    def __init__(self, name, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level)
        self.node_name = name
        self.rgb_ring = RGBRing(5)
        self.set_mode_sub = rospy.Subscriber(self.node_name + "/set_mode", std_msgs.msg.UInt8, self.set_mode_callback)
        self.set_breath_rate_sub = rospy.Subscriber(self.node_name + "/set_breath_rate", std_msgs.msg.Float32, self.set_breath_rate_callback)
        self.set_color_sub = rospy.Subscriber(self.node_name + "/set_color", std_msgs.msg.UInt8MultiArray, self.set_color_callback)
        self.timer = rospy.Timer(rospy.Duration(0.03), self.timer_callback)
        self.update = self.rgb_ring.mode1_update
    
    def timer_callback(self, _):
        if self.update is not None:
            self.update()
    
    def set_mode_callback(self, msg):
        rospy.logdebug(str(msg))
        if hasattr(self.rgb_ring, 'mode' + str(msg.data) + '_update'):
            self.update = getattr(self.rgb_ring, 'mode' + str(msg.data) + '_update')
    
    def set_breath_rate_callback(self, msg):
        rospy.logdebug(str(msg))
        print(msg)
        if msg.data == 0:
            incs = [0] * self.rgb_ring.pixels_num
        else:
            inc = 360 / ((1.0 / msg.data) * (1.0 / 0.03))
            incs = [inc, ] * self.rgb_ring.pixels_num
        self.rgb_ring.values_inc = incs

    def set_color_callback(self, msg):
        rospy.logdebug(str(msg))
        r, g, b = msg.data[:3]
        buf = cv2.cvtColor(np.array([[[r, g, b], ], ], dtype=np.uint8), cv2.COLOR_RGB2HSV).reshape(3)
        self.rgb_ring.color_indexs = [buf[0],] * self.rgb_ring.pixels_num
        self.rgb_ring.saturation = [buf[1],] * self.rgb_ring.pixels_num
        self.rgb_ring.values = [buf[2],] * self.rgb_ring.pixels_num


def main():
    try:
        rgb_ring_node = RGBRingNode('rgb_ring', log_level=rospy.DEBUG)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == '__main__':
    main()

