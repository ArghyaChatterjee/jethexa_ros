#!/usr/bin/env python3
# read the servo status
import argparse
import rospy
import sys
import time
import time
from jethexa_sdk import serial_servo
import ctypes


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('servo_id',  metavar="SERVO ID", nargs='?', type=int, help="舵机 ID", default=1) # add servo id parameter
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    servo_id = argv.servo_id if type(argv.servo_id) is int else argv.servo_id[0]
    assert(servo_id > 0)


    serial_servo.load_or_unload_write(servo_id, 0) # NO.3 servo power off

    print("\x1b[?25l") # hide cursor and add a blank line
    count = 0
    try:
        while True:
            count += 1
            print("count:%d" % count)
            print("VOLT:{:0.2f}v".format(serial_servo.read_vin(servo_id) / 1000.0)) # read the current position of NO.3 servo 
            print("POSITION:%d" % serial_servo.read_pos(servo_id)) # read the current supply voltage of N0.3 servo
            print('TEMPERATURE:%d°' % serial_servo.read_temp(servo_id))
            print('DEVIATION:%d' % ctypes.c_int8(serial_servo.read_deviation(servo_id)).value)
            limit = serial_servo.read_angle_limit(servo_id)
            print('POSITION ANGE:%d-%d' % (limit[0], limit[1]))
            limit = serial_servo.read_vin_limit(servo_id)
            print('VOLTAGE INPUT LIMIT:%dmv-%dmv' % (limit[0], limit[1]))
            limit = serial_servo.read_temp_limit(servo_id)
            print('TEMPERATURE LIMIT:50°-%d°' % limit)
            print("\x1b[8A\r", end='') # Go back to the cursor and move up 8 lines, that is, go back to the first line
            time.sleep(0.2)
    except Exception as e:
        print(e)