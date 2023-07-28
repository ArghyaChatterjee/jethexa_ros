import time
import Jetson.GPIO as GPIO

PIN = 18
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.OUT)


def on():
    GPIO.output(PIN, 1)


def off():
    GPIO.output(PIN, 0)


def beep(interval, repeat, sleep=time.sleep):
    for _ in range(repeat):
        on()
        sleep(interval)
        off()
        sleep(interval)
