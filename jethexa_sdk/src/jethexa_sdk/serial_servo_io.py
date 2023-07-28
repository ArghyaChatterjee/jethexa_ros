#!/usr/bin/python3

import serial
import time
import ctypes
import sys
import Jetson.GPIO as gpio

LOBOT_SERVO_FRAME_HEADER = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE = 1
LOBOT_SERVO_MOVE_TIME_READ = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ = 8
LOBOT_SERVO_MOVE_START = 11
LOBOT_SERVO_MOVE_STOP = 12
LOBOT_SERVO_ID_WRITE = 13
LOBOT_SERVO_ID_READ = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE = 18
LOBOT_SERVO_ANGLE_OFFSET_READ = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE = 20
LOBOT_SERVO_ANGLE_LIMIT_READ = 21
LOBOT_SERVO_VIN_LIMIT_WRITE = 22
LOBOT_SERVO_VIN_LIMIT_READ = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ = 25
LOBOT_SERVO_TEMP_READ = 26
LOBOT_SERVO_VIN_READ = 27
LOBOT_SERVO_POS_READ = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ = 32
LOBOT_SERVO_LED_CTRL_WRITE = 33
LOBOT_SERVO_LED_CTRL_READ = 34
LOBOT_SERVO_LED_ERROR_WRITE = 35
LOBOT_SERVO_LED_ERROR_READ = 36

serialHandle = serial.Serial("/dev/ttyTHS1", 115200)  # Initialize the serial port, and the baud rate is 115200

gpio.setwarnings(False)
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)


def portInit():  # IO port used for configuration
    gpio.setup(17, gpio.OUT, initial=0)  # Configure RX_CON as GPIO17 as output
    gpio.setup(27, gpio.OUT, initial=1)  # Configure TX_CON as GPIO27 as output


portInit()


def portWrite():  # Configure single-wire serial port as output
    gpio.output(27, 1)  # Pull high TX_CON i.e. GPIO27
    gpio.output(17, 0)  # Pull down RX_CON i.e. GPIO17


def portRead():  # Configure single-wire serial port as input
    gpio.output(17, 1)  # Pull high RX_CON i.e. GPIO17
    gpio.output(27, 0)  # Pull down TX_CON i.e. GPIO27


def portRest():
    time.sleep(0.1)
    serialHandle.close()
    gpio.output(17, 1)
    gpio.output(27, 1)
    serialHandle.open()
    time.sleep(0.1)


def checksum(buf):
    # Calculate checksum
    sum = 0x00
    for b in buf:  # sum
        sum += b
    sum = sum - 0x55 - 0x55  # Remove the two 0x55s at the beginning of the command
    sum = ~sum  # negate
    return sum & 0xff

def serial_servo_write_multi_cmd(cmd, data):
    '''
    同时向多个舵机写指令
    '''
    portWrite()
    bufs = []
    for servo_id, dat1, dat2 in data:
        buf = bytearray(b'\x55\x55')  # frame header
        buf.append(servo_id)
        # command length 
        if dat1 is None and dat2 is None:
            buf.append(3)
        elif dat1 is not None and dat2 is None:
            buf.append(4)
        elif dat1 is not None and dat2 is not None:
            buf.append(7)
        # command
        buf.append(cmd)
        # write data
        if dat1 is None and dat2 is None:
            pass
        elif dat1 is not None and dat2 is None:
            buf.append(dat1 & 0xff)  # deviation
        elif dat1 is not None and dat2 is not None:
            buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # Divide the lower 8 bits and the upper 8 bits into the cache
            buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # Divide the lower 8 bits and the upper 8 bits into the cache
        # checksum
        buf.append(checksum(buf))
        bufs.extend(buf)
    serialHandle.write(bufs)


def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    写指令
    :param id:
    :param w_cmd:
    :param dat1:
    :param dat2:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # frame header
    buf.append(id)

    # command length
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    # command 
    buf.append(w_cmd)

    # write data
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # deviation
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # Divide the lower 8 bits and the upper 8 bits into the cache
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # Divide the lower 8 bits and the upper 8 bits into the cache

    # checksum
    buf.append(checksum(buf))

    # for b in buf:
    #     print(int(b), end=', ')
    # print()
    # send
    #print(buf)
    serialHandle.write(buf)


def serial_servo_read_cmd(id=None, r_cmd=None):
    '''
    发送读取命令
    :param id:
    :param r_cmd:
    :param dat:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # frame header
    buf.append(id)
    buf.append(3)  # command length
    buf.append(r_cmd)  # command
    buf.append(checksum(buf))  # checksum
    serialHandle.write(buf)  # send
    time.sleep(0.00034)


def serial_servo_get_rmsg(cmd):
    '''
    # Get the data of the specified read command
    :param cmd: 读取命令
    :return: 数据
    '''
    serialHandle.flushInput()  # Clear receive cache
    portRead()  # Configure single-wire serial port as input
    time.sleep(0.005)  # After a little delay, wait for the reception to finish
    count = serialHandle.inWaiting()  # Get the number of bytes in the receive buffer
    if count != 0:  # If the received data is not empty
        recv_data = serialHandle.read(count)  # read received data
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # Clear receive cache
                if dat_len == 4:
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                return None
        except BaseException as e:
            return None
            print(e)
    else:
        serialHandle.flushInput()  # Clear receive cache
        return None
