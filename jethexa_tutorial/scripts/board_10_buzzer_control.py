#!/usr/bin/env python3

import time
from jethexa_sdk import buzzer


if __name__ == "__main__":
    buzzer.on()     # buzzer beeps
    time.sleep(0.1) 
    buzzer.off()    # buzzer stops
    time.sleep(0.5)

    buzzer.on()  # buzzer beeps
    time.sleep(0.5)
    buzzer.off() # buzzer stops
    time.sleep(1)

    for i in range(5):
        buzzer.on()     # buzzer beeps
        time.sleep(0.1) 
        buzzer.off()    # buzzer stops
        time.sleep(0.1)





