#!/usr/bin/env python3

import sys
import time
from jethexa_sdk import serial_servo


if __name__ == "__main__":

    serial_servo.set_position(1, 500, 1000) # Turn the No. 1 servo to the 500 position in 1000 milliseconds, and 0 - 1000 corresponds to 0 - 240 degrees
    time.sleep(1)

    serial_servo.set_position(1, 300, 2000) # Turn the No. 1 servo to the 500 position in 2000 milliseconds, and 0 - 1000 corresponds to 0 - 240 degrees
    time.sleep(2)

    serial_servo.set_position(1, 700, 2000) # Turn the No. 1 servo to the 500 position in 2000 milliseconds, and 0 - 1000 corresponds to 0 - 240 degrees
    time.sleep(2)

    serial_servo.set_position(1, 500, 1000) # Turn the No. 1 servo to the 500 position in 1000 milliseconds, and 0 - 1000 corresponds to 0 - 240 degrees
    time.sleep(1)

    sys.exit(0)