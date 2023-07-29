#!/usr/bin/env python
# coding=utf-8
import rospy
import smbus
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis
from std_srvs.srv import Empty, EmptyResponse
import struct

ADDR = None
bus = None

PWR_MGMT_1 = 0x6b

GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40
TEMP_H = 0x41
TEMP_L = 0x42
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48


def read_block_data(adr, count):
    data = bus.read_i2c_block_data(ADDR, adr, count)
    return data


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

def publish_temp(timer_event):
    global publish_enable, temp
    if not publish_enable:
        return
    try:
        temp_msg = Temperature()
        temp_msg.header.frame_id = imu_link
        temp_msg.temperature = temp #read_word_2c(TEMP_H)/340.0 + 36.53
        temp_msg.header.stamp = rospy.Time.now()
        temp_pub.publish(temp_msg)
    except Exception as e:
        pass

def publish_imu(timer_event):
    global publish_enable, temp
    if not publish_enable:
        return
    try:
        imu_msg = Imu()
        imu_msg.header.frame_id = imu_link
        data = bytearray(read_block_data(ACCEL_XOUT_H, 14))
        data = struct.unpack(">hhhhhhh", data)
        print(data)

        # Read the acceleration vals
        accel_x = data[0] / 16384.0 #read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = data[1] / 16384.0 #read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = data[2] / 16384.0 #read_word_2c(ACCEL_ZOUT_H) / 16384.0

        # temp
        temp = data[3] / 340.0 + 36.53

        # Read the gyro vals
        gyro_x = data[4] / 131.0 #read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = data[5] / 131.0 #read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = data[6] / 131.0 #read_word_2c(GYRO_ZOUT_H) / 131.0

        # 要求按 ENU 格式发布数据(参考rep-103) 这里需要根据实际情况转换一下
        # https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions
        imu_msg.linear_acceleration.x = -accel_y*9.8 
        imu_msg.linear_acceleration.y = accel_x*9.8
        imu_msg.linear_acceleration.z = accel_z*9.8

        imu_msg.angular_velocity.x = -gyro_y*0.0174
        imu_msg.angular_velocity.y = gyro_x*0.0174
        imu_msg.angular_velocity.z = gyro_z*0.0174

        imu_msg.header.stamp = rospy.Time.now()

        imu_pub.publish(imu_msg)
    except Exception as e:
        #rospy.logerr(e)
        pass

def start_pub_cb(_):
    global publish_enable
    publish_enable = True
    return EmptyResponse()


def stop_pub_cb(_):
    global publish_enable
    publish_enable = False
    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    imu_link = rospy.get_param('~imu_link', 'imu_link')
    imu_hz = rospy.get_param('~imu_hz', 50)
    imu_topic = rospy.get_param('~imu_topic', 'imu/raw')
    publish_enable = rospy.get_param("~start_publish", True)
    temp_topic = rospy.get_param('~temperature_topic', 'imu/temperature')
    temp = 0

    while True:
        try:
            bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
            break
        except Exception as e:
            rospy.logerr(str(e))

    start_pub_srv =  rospy.Service('start_imu', Empty, start_pub_cb)
    stop_pub_srv =  rospy.Service('stop_imu', Empty, stop_pub_cb)
    temp_pub = rospy.Publisher(temp_topic, Temperature, queue_size=2)
    imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=2)
    imu_timer = rospy.Timer(rospy.Duration(1.0 / imu_hz), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()

