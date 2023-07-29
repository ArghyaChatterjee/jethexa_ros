#!/usr/bin/env python3

import rospy
import smbus
import time

ADDR = None
bus = None
IMU_FRAME = None

PWR_MGMT_1 = 0x6b

ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

TEMP_H = 0x41
TEMP_L = 0x42

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


def read_mpu6050():
    temperature = read_word_2c(TEMP_H)/340.0 + 36.53

    # Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
    
    # Read the gyro vals
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0

    print(
    """
    TEMP:{:0.2f}
    ACCEL X:{:>9.4f}  Y:{:>9.4f}  Z:{:>9.4f}
    GYRO  X:{:>9.4f}  Y:{:>9.4f}  Z:{:>9.4f}
    """.format(temperature, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z))
    print("\x1b[6A\r") # return to the first line
    
if __name__ == '__main__':
    bus = smbus.SMBus(1)
    ADDR = 0x68

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
    try:
        print("\x1b[?25l", end='\r')
        while True:
            try:
                read_mpu6050()
            except KeyboardInterrupt:
                break
            except Exception as e:
                pass
            time.sleep(0.1)
    except Exception as e:
        print(e)
