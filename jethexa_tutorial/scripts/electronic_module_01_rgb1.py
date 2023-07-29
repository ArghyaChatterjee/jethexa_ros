#!/usr/bin/env python3

import argparse
import sys
import time
import cv2
import numpy as np
import board
import neopixel_spi as neopixel
import rospy


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("color",  metavar="HEX_RGB_COLOR", nargs="?", type=str, help="十六进制RGB颜色", default="0xFFFF00") #add colors
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    try:
        color = int(argv.color, 16) #Convert hexadecimal string to number
    except Exception as e:
        print(e)
        sys.exit(-1)

    spi = board.SPI()
    pixels_num = 5
    pixels = neopixel.NeoPixel_SPI(spi, pixels_num, pixel_order=neopixel.GRB, auto_write=False)
    value_inc = -7 # Increment value for brightness change

    #  Separate rgb color values into separate components
    r, g, b = (color >> 16) & 0x0000FF, (color >> 8) & 0x0000FF, color & 0x0000FF # opencv rgb range; r: 0-255, g:0-255, b:0-255

    # We need a breathing light, that is, the color does not change, only the brightness changes, which can be achieved with the hsv color space
    # Create a 1-pixel opencv image with rgb values, then convert the image from rgb to hsv with opencv to implement color space conversion
    # opencv hsv range; h: 0-180， s: 0-255， v: 0-255
    h, s, v = cv2.cvtColor(np.array([[[r, g, b]]], dtype=np.uint8), cv2.COLOR_RGB2HSV).reshape(3)

    try:
        while True:
            v += value_inc # change brightness
            value_inc = -value_inc if v > 180 or v < 0 else value_inc # Flip the delta value when v tops or bottoms
            v = max(min(v, 180), 0) # limit the range of v to 0-180

            # change hsv into rgb
            r, g, b = cv2.cvtColor(np.array([[[h, s, v]]], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
            data = [int(r << 16 | g << 8 | b)] * pixels_num # Combine rgb components into 24bit color values
            # pixels = data is not correct here, because pixels is not a simple list, but an rgb light strip object
            for i in range(pixels_num):
                pixels[i] = data[i] # Set the cache for each rgb lamp
            pixels.show() # write cache to lightbulb
            time.sleep(0.03) # 0.03 seconds. 30 refreshes per second
    except Exception as e:
        print(e)
