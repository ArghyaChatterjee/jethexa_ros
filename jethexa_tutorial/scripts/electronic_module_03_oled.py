#!/usr/bin/env python3
# oled_helloworld.py realizes that oled displays hellowolrd

import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import Adafruit_SSD1306


if __name__ == "__main__":
    # Initialize the OLED and related resources
    screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
    screen.begin() # start the screen

    # Variable resources used to implement functions
    # font = ImageFont.load_default() # load default font
    # load ttf font
    font_1 = ImageFont.truetype('/home/hiwonder/jethexa/src/jethexa_tutorial/misc/wqy-MicroHei.ttf', 22)

    screen.image(Image.fromarray(np.zeros((32, 128), np.uint8)).convert('1')) # Build 0 cache
    screen.display() # Display 0 cache, that is to clear the screen

    buf = Image.new('1', (screen.width, screen.height)) # Create an Image object for drawing
    draw = ImageDraw.Draw(buf) # build canvas
    try:
        while True:
            draw.rectangle((0, 0, screen.width, screen.height), outline=0, fill=0) # Filling a black box is to clear the cache to black
            draw.text((1, 5),   "Hello World!", font=font_1, fill=255) # Drawing Hello World on canvas
            screen.image(buf) # write our graph to the cache
            screen.display() # write cache to screen
            time.sleep(1)
            draw.rectangle((0, 0, screen.width, screen.height), outline=0, fill=0) # clear
            draw.text((12, 5),   "你好 世界!", font=font_1, fill=255) # write chiness
            screen.image(buf) # write graph to cache
            screen.display() # write cache to screen
            time.sleep(1)
    except Exception as e:
        print(e)

