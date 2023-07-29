#!/usr/bin/env python3

import sys
import time
from jethexa_sdk import serial_servo


if __name__ == "__main__":

    serial_servo.set_position(1, 500, 1000) # spend 1000ms rotating NO.1 servo to 500 position. 0-1000 position corresponds to 0-240 degrees.
    time.sleep(2)

    current_pos = 500
    current_angle = 0 # set the current angle as 0 degree
    new_angle = 30 # the new angle is 30 degree
    new_pos = current_pos + (new_angle - current_angle) * (1000 / 240) # Calculate the servo angle value corresponding to the new angle 
 
    serial_servo.set_position(1, int(new_pos), 2000) # Rotate 30 degrees in 2000 milliseconds, that is, the angular velocity is 15 degrees/second
    time.sleep(2)

    serial_servo.set_position(1, 500, 1000) # Turn the No. 1 servo to the 500 position in 1000 milliseconds, that is, the angular velocity is 30 degrees/second
    time.sleep(2)
    # The angle conversion relationship can be encapsulated as a function to directly control the rotation of the steering gear at a certain angular speed