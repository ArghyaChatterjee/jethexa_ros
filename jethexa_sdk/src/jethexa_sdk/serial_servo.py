import time
from .serial_servo_io import *


def set_position(servo_id, pos, duration):
    """
    Set servo position(angle)

    :param servo_id: the id of the servo you want to set
    :param pos: the goal position
    :param duration:
    :return: None
    """
    pos = 1000 if pos > 1000 else pos
    pos = 0 if pos < 0 else pos
    duration = 0 if duration < 0 else duration
    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_MOVE_TIME_WRITE, pos, duration)


def set_multi_position(data):
    '''
    Set mulit servo position
    '''
    data_n = []
    for servo_id, pos, duration in data:
        pos = 1000 if pos > 1000 else pos
        pos = 0 if pos < 0 else pos
        duration = 0 if duration < 0 else duration
        data_n.append((servo_id, pos, duration))
    serial_servo_write_multi_cmd(LOBOT_SERVO_MOVE_TIME_WRITE, data_n)


def set_deviation(servo_id, d):
    """
    Set servo deviation

    :param servo_id:
    :param d:
    :return:
    """
    if servo_id < 1:
        return
    if d < -200 or d > 200:
        return
    if serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d) is None:
        pass


def stop(servo_id=None):
    """
    Stop servo

    :param servo_id:
    :return:
    """
    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_MOVE_STOP)


def set_id(old_id, new_id):
    """
    Set servo id
    :param old_id:
    :param new_id:
    """
    serial_serro_wirte_cmd(old_id, LOBOT_SERVO_ID_WRITE, new_id)


def read_id(servo_id=None, count=100):
    """
    Read servo id

    :param servo_id:
    :param count:
    :return:
    """
    while count > 0:
        count -= 1
        if servo_id is None:  # only one servo connected to the bus
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(servo_id, LOBOT_SERVO_ID_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg


def save_deviation(servo_id):
    """
    storage the deviation
    :param servo_id: the id of the servo you want to control
    :return:
    """
    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)


def read_deviation(servo_id, count=100):
    """
    Read deviation from servo

    :param servo_id: the id of the servo you want to read
    :param count:
    :return: deviation
    """
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        if msg is not None:
            return msg


def set_angle_limit(servo_id, low, high):
    """
    :param servo_id:
    :param low:
    :param high:
    :return:
    """
    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)


def read_angle_limit(servo_id, count=100):
    """
    :param servo_id:
    :param count:
    :return:
    """
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            return msg


def set_vin_limit(servo_id, low, high):
    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)


def read_vin_limit(servo_id, count=100):
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg


def set_max_temp(servo_id, m_temp):
    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)


def read_temp_limit(servo_id, count=100):
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg


def read_pos(servo_id, count=100):
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def read_position(servo_id, count=100):
    return read_pos(servo_id, count)

def read_temp(servo_id, count=100):
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg


def read_vin(servo_id, count=100):
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg


def reset_pos(old_id):
    """
    :param old_id:
    """
    set_deviation(old_id, 0)  # clear deviation
    time.sleep(0.1)
    serial_serro_wirte_cmd(old_id, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)  # middle point


def load_or_unload_write(servo_id, new_state):
    serial_serro_wirte_cmd(servo_id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, new_state)


def load_or_unload_read(servo_id, count=100):
    while count > 0:
        count -= 1
        serial_servo_read_cmd(servo_id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            return msg


def show_servo_state():
    old_id = read_id()
    portRest()
    if old_id is not None:
        print('ID：%d' % old_id)
        pos = read_pos(old_id)
        print('Position：%d' % pos)
        portRest()

        now_temp = read_temp(old_id)
        print('temperature：%d°' % now_temp)
        portRest()

        now_vin = read_vin(old_id)
        print('voltage input：%dmv' % now_vin)
        portRest()

        d = read_deviation(old_id)
        print('deviation：%d' % ctypes.c_int8(d).value)
        portRest()

        limit = read_angle_limit(old_id)
        print('position range:%d-%d' % (limit[0], limit[1]))
        portRest()

        vin = read_vin_limit(old_id)
        print('voltage range:%dmv-%dmv' % (vin[0], vin[1]))
        portRest()

        temp = read_temp_limit(old_id)
        print('temperature limit:50°-%d°' % temp)
        portRest()
    else:
        print('Read id fail')


if __name__ == '__main__':
    show_servo_state()
