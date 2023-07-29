#!/usr/bin/env python3

import time
from jethexa_sdk import pwm_servo

if __name__ == "__main__":
    pwm_servo.pwm_servo1.start()  # start NO.1 pwm control thread
    pwm_servo.pwm_servo2.start()  # start NO.2 pwm control thread

    pwm_servo.pwm_servo1.set_position(1500, 1000)  # No. 1 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    pwm_servo.pwm_servo2.set_position(1500, 1000)  # No. 2 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    time.sleep(1)

    pwm_servo.pwm_servo1.set_position(500, 1000)  # No. 1 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    pwm_servo.pwm_servo2.set_position(2500, 1000)  # No. 2 pwm servo rotates to the 2500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    time.sleep(1) 

    pwm_servo.pwm_servo1.set_position(2500, 2000)  # No. 1 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    pwm_servo.pwm_servo2.set_position(500, 2000)  # No. 2 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    time.sleep(2)

    pwm_servo.pwm_servo1.set_position(1500, 1000)  # No. 1 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    pwm_servo.pwm_servo2.set_position(1500, 1000)  # No. 2 pwm servo rotates to the 1500 value position in 1000 milliseconds, the servo value 500~2500 corresponds to 0 ~ 180 degrees
    time.sleep(1)