#!/usr/bin/env python3

import sys
import time
from jethexa_sdk import pwm_servo

if __name__ == "__main__":
    pwm_servo.pwm_servo1.start()  # start NO.1 pwm control thread
    pwm_servo.pwm_servo2.start()  # start NO.2 pwm control thread

    pwm_servo.pwm_servo1.set_position(1500, 1000)  # No. 1 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    time.sleep(1)

    current_pos = 1500
    current_angle = 0 # set the current angle as 0 degree
    new_angle = 90 # the new angle is 90 degree
    new_pos = current_pos + (new_angle - current_angle) * ((2500 - 1500) / 180) # calculate the servo angle value corresponding to the new angle

    pwm_servo.pwm_servo1.set_position(int(new_pos), 2000) # It takes 2000 milliseconds to turn 90 degrees, that is, the angular velocity is 45 degrees/second
    time.sleep(2)


    pwm_servo.pwm_servo1.set_position(1500, 1000) # Turn the No. 1 servo to the position of 1500 in 1000 milliseconds, that is, the angular velocity is 90 degrees/second
    # The angle conversion relationship can be encapsulated as a function to directly control the rotation of the steering gear at a certain angular speed
    time.sleep(2)

        # The pwm resolution of jetson nano is too low, so the action will be discontinuous
