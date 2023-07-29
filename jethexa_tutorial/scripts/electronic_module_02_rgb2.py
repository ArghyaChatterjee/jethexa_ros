#!/usr/bin/env python3

import time
import cv2
import board
import numpy as np
import neopixel_spi as neopixel


if __name__ == "__main__":
    spi = board.SPI()
    pixels_num = 5
    pixels = neopixel.NeoPixel_SPI(spi, pixels_num, pixel_order=neopixel.GRB, auto_write=False)
    pixel_index = pixels_num - 1
    color_index = 0 # color value of hsv, 0-180
    color_inc = 5

    try:
        while True:
            color_index += 5 # change color
            if color_index >= 180: # the top of color
                color_index = color_index % 180 # new color value. Modulo
            # Calculate the color value of the light one by one, and set the bulb color buffer. A circle covers the entire 0-180 range
            for i in range(pixels_num):
                # The hue interval of each bulb is 180/ the number of bulbs, and the colors between the bulbs will be continuous due to scattering color mixing
                c_index = (color_index + int(180 / pixels_num) * i) % 180
                # Create an opencv image in the hsv color space with color hue values, saturation and lightness of 255
                # Convert hsv to rgb with opencv
                r, g, b = cv2.cvtColor(np.array([[[c_index, 255, 255], ],], dtype=np.uint8), cv2.COLOR_HSV2RGB).reshape(3)
                pixels[i] = int(r << 16 | g << 8 | b) # Combine rgb values into 24bit color values
            pixels.show()
            time.sleep(0.03) # 0.03 seconds. Refresh 30 times per second.
    except Exception as e:
        print(e)
