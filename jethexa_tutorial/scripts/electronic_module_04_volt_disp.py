#!/usr/bin/env python3
# realize that oled screen displays servo voltage and position

import argparse
import rospy
import sys
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import Adafruit_SSD1306
import time
from jethexa_sdk import serial_servo


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('servo_id',  metavar="SERVO ID", nargs='?', type=int, help="舵机 ID", default=1) # add servo id parameter
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    servo_id = argv.servo_id if type(argv.servo_id) is int else argv.servo_id[0]
    assert(servo_id > 0)

    # Initialize the OLED and related resources
    screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
    screen.begin() # start the screen

    # Variable resources used to implement functions
    # font = ImageFont.load_default() # load the default font
    # load ttf font
    font_1 = ImageFont.truetype('/home/hiwonder/jethexa/src/jethexa_tutorial/misc/wqy-MicroHei.ttf', 16)

    screen.image(Image.fromarray(np.zeros((32, 128), np.uint8)).convert('1')) # Build 0 cache
    screen.display() # Display 0 cache, that is to clear the screen

    buf = Image.new('1', (screen.width, screen.height)) # Create an Image object for drawing
    draw = ImageDraw.Draw(buf) # build canvas

    serial_servo.load_or_unload_write(servo_id, 0) # servo_id servo unloads. we can twist No. 3 servo to test the position to show whether it changes

    try:
        while True:
            volt_str = "电压: {:0.2f} v".format(serial_servo.read_vin(servo_id) / 1000.0) # Read the current power supply voltage of servo_id number
            position_str = "位置: %d" % serial_servo.read_pos(servo_id)  # Read the position of the servo with servo_id number

            draw.rectangle((0, 0, screen.width, screen.height), outline=0, fill=0) # Filling a black box is to clear the cache to black
            draw.text((1, 0), volt_str, font=font_1, fill=255) # write Hello World on canvas
            draw.text((1, 15), position_str, font=font_1, fill=255) # write Hello World on canvas
            screen.image(buf) # write our graph to the cache
            screen.display()
            time.sleep(0.2)

    except Exception as e:
        print(e)




