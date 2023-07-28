#!/usr/bin/env python3

import os
import sys
import time
import math
import rospy
import threading
import pygame as pg
from enum import Enum
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from jethexa_sdk import misc, buzzer
from jethexa_controller import client


BIG_STEP = 0.1
BIG_ROTATE = math.radians(30)
SMALL_STEP = 0.06
SMALL_ROTATE = math.radians(15)


AXES_MAP = 'ly', 'lx', 'ry', 'rx'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', 'hat_xu', 'hat_xd', 'hat_yl', 'hat_yr', 'laxis_l', 'laxis_r', 'laxis_u', 'laxis_d', 'raxis_l', 'raxis_r', 'raxis_u', 'raxis_d'


class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3


class JoystickControlNode:
    def __init__(self, name):
        os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
        pg.display.init()

        rospy.init_node(name, anonymous=True)
        self.jethexa = client.Client(self)

        self.last_axes =  dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.transform = [[0, 0, 0], [0, 0, 0]]
        self.direction, self.rotate, self.stride = 0, 0, 0
        self.period = 1.0
        self.step_height = 20
        self.gait = 2
        self.count = 0
        self.joy = None
        self.do_transform = False
        self.do_movement = False
        self.set_parameters = False
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.joy_callback)

    def axes_callback(self, axes):
        pass

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            period = self.period - 0.1
            self.period = max(period, 0.8)
            self.set_parameters = True
            self.do_movement = True

    def l2_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            period = self.period + 0.1
            self.period = min(period, 3.0)
            self.set_parameters = True
            self.do_movement = True

    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            step_height = self.step_height + 2
            self.step_height = min(step_height, 50)
            self.set_parameters = True
            self.do_movement = True

    def r2_callback(self, new_state):
        if new_state == ButtonState.Pressed or new_state == ButtonState.Holding:
            step_height = self.step_height - 2
            self.step_height = max(step_height, 14)
            self.set_parameters = True
            self.do_movement = True

    def cross_callback(self, new_state):
        if new_state == ButtonState.Holding:
            self.transform[1][1] = math.radians(-2)
            self.do_transform = True

    def triangle_callback(self, new_state):
        if new_state == ButtonState.Holding:
            self.transform[1][1] = math.radians(2)
            self.do_transform = True

    def circle_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.rotate, self.stride = 0, -SMALL_ROTATE, 0
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def square_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.rotate, self.stride = 0, SMALL_ROTATE, 0
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def raxis_u_callback(self, new_state):
        if new_state == ButtonState.Holding:
            self.transform[0][2] = 5
            self.do_transform = True


    def raxis_d_callback(self, new_state):
        if new_state == ButtonState.Holding:
            self.transform[0][2] = -5
            self.do_transform = True

    def raxis_l_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 1
            self.direction, self.rotate, self.stride = 0, BIG_ROTATE, 0
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def raxis_r_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 1
            self.direction, self.rotate, self.stride = 0, -BIG_ROTATE, 0
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.rotate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def laxis_u_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 1
            self.direction, self.roatate, self.stride = 0, 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def laxis_d_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 1
            self.direction, self.roatate, self.stride = math.radians(180), 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def laxis_l_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 1
            self.direction, self.roatate, self.stride = math.radians(90), 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def laxis_r_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 1
            self.direction, self.roatate, self.stride = math.radians(270), 0, BIG_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def hat_xu_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.roatate, self.stride = 0, 0, SMALL_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def hat_xd_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.roatate, self.stride = math.radians(180), 0, SMALL_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def hat_yl_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.roatate, self.stride = math.radians(90), 0, SMALL_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def hat_yr_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.gait = 2
            self.direction, self.roatate, self.stride = math.radians(270), 0, SMALL_STEP
            self.do_movement = True
        elif new_state == ButtonState.Released:
            self.direction, self.roatate, self.stride = 0, 0, 0
            self.do_movement = True
        else:
            pass

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            if self.last_buttons['select'] == 1:
                print("A")
                self.direction, self.rotate, self.stride = 0, 0, 0
                self.period = 1.0
                self.step_height = 20
                self.gait = 2
                self.count = 0
                buzzer.beep(0.2, 1, rospy.sleep)
            self.jethexa.traveling(gait=-2, time=1, steps=0)

    def joy_callback(self, event):
        # 检查当前是否插入
        self.count += 1
        if self.count > 10:
            if os.path.exists("/dev/input/js0"):
                if self.joy is None:
                    rospy.sleep(1)
                    pg.joystick.init()
                    if pg.joystick.get_count() > 0:
                        try:
                            self.joy = pg.joystick.Joystick(0)
                            self.joy.init()
                        except Exception as e:
                            rospy.logerr(e)
            else:
                if self.joy is not None:
                    try:
                        self.joy.quit()
                        pg.joystick.quit()
                        self.joy = None
                    except Exception as e:
                        rospy.logerr(e)

        if self.joy is None:
            return

        pg.event.pump()
        buttons = list(self.joy.get_button(i) for i in range(15))
        hat = list(self.joy.get_hat(0))
        axes = list(-self.joy.get_axis(i) for i in range(4))

        # 摇杆值
        axes = dict(zip(AXES_MAP, axes))
        #print(axes)
        #print(axes)
        # 摇杆 ad 值转为 bool 值
        laxis_l, laxis_r = 1 if axes['ly'] > 0.5 else 0, 1 if axes['ly'] < -0.5 else 0
        laxis_u, laxis_d = 1 if axes['lx'] > 0.5 else 0, 1 if axes['lx'] < -0.5 else 0
        raxis_l, raxis_r = 1 if axes['ry'] > 0.5 else 0, 1 if axes['ry'] < -0.5 else 0
        raxis_u, raxis_d = 1 if axes['rx'] > 0.5 else 0, 1 if axes['rx'] < -0.5 else 0

        # 方向帽的 ad 值转为 bool 值
        hat_y, hat_x = hat
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yd, hat_yu = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0

        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd])
        buttons.extend([laxis_l, laxis_r, laxis_u, laxis_d, raxis_l, raxis_r, raxis_u, raxis_d]) 
        buttons = dict(zip(BUTTON_MAP, buttons))

        # 判断摇杆值的是否改变
        axes_changed = False
        for key, value in axes.items(): # 轴的值被改变
            if self.last_axes[key] != value:
                axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))

        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                # rospy.loginfo(key + ': ' + str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))

        #if self.set_parameters:
        #    self.jethexa.traveling(10 + self.gait, height=self.step_height, time=self.period)
        #    self.set_parameters = False

        if self.do_movement:
            self.jethexa.traveling(10 + self.gait, height=self.step_height, time=self.period)
            speed = self.stride * (1.0 / self.period)
            rotate = self.rotate * (1.0 / self.period)
            vx = math.cos(self.direction) * speed
            vy = math.sin(self.direction) * speed
            #print(round(vx, 4), round(vy, 4))
            self.jethexa.cmd_vel(vx, vy, rotate)
            self.do_movement = False

        if self.do_transform:
            self.jethexa.pose_transform_euler(self.transform[0], self.transform[1], 0.1)
            self.transform = [[0, 0, 0], [0, 0, 0]]
            self.do_transform = False

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickControlNode('joystick_control')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

