#!/usr/bin/env python3

# this file is one part of JetHexa
# oled_display.py realizes oled displaying system status, sensor parameters, etc. 

import sys
import os
import signal
import subprocess

import threading
from cv2 import invert
import rospy
import rospkg
import std_msgs.msg
import std_srvs.srv
import numpy as np
import queue
import psutil

from PIL import Image, ImageDraw, ImageFont

import time
import Adafruit_SSD1306
from vision_utils import fps
from wireless_utils import dev_state
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
from button_helper import ButtonHelper


class Cutscens:
    def __init__(self, direction, increment, new_gram_name):
        self.direction = direction
        self.increment = increment
        self.new_gram_name = new_gram_name
        self.now_gram = None
        self.pixel_index = 0
    

class OledDisplayNode:
    def __init__(self, name, anonymous=True, log_level=rospy.INFO):
        # Button initialization. Press button to switch screen
        GPIO.setmode(GPIO.BCM) 
        GPIO.setup(4, GPIO.IN)
        self.btn = ButtonHelper(read_button=lambda : GPIO.input(4), clicked_cb=self.clicked_callback)

        rospy.init_node(name, log_level=log_level)
        self.node_name = name
        self.voltage = 0.0
        self.voltage_sub = rospy.Subscriber('/voltage', Float32, self.voltage_update, queue_size=1)

        # initialize OLED and related resources
        self.screen = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1, i2c_address=0x3C)
        self.screen.begin() # start the screen
        self.fps = fps.FPS() # screen fps stats

        # Variable resources used to implement functions
        self.lock = threading.RLock()
        self.on_off = True # whether to turn on screen display
        self.refresh_enable = True # whether to refresh the screen regularly

        self.last_gram = None # the last data output on the screen
        self.current_gram_name = "clear" # current cache name displayed
        self.current_cutscenes = None # currently executing cutscenes
        self.cutscenes = queue.Queue(maxsize=1) # cutscene queue
        self.font = ImageFont.load_default() # load the font

        # Timed inverse display to prevent burn-in
        self.invert = False # Whether to display in reverse
        self.invert_interval = 30 # Actual interval of timed inversion
        self.invert_timestamp = time.time() # timestamp of last inversion

        # Create a cache for each screen
        self.grams = {"clear": np.zeros((32, 128), dtype=np.uint8), # all-black image
                      "full": np.full((32, 128), 255, dtype=np.uint8)} # all-white image
        self.load_logo() # load logo
        self.sys_states_update(0) 
        self.wifi_iface = rospy.get_param('~wifi_iface', 'wlan0') # Get the name of the device to display the wireless card
        self.wireless_states_update(0)

        # Add a cutscene that shows the logo and let the logo fade in the oled from bottom to top
        self.cutscenes.put(Cutscens('bt', 1, 'logo')) 
        # Add a timer to let the system status information fade in the oled after 3.5 seconds
        rospy.Timer(rospy.Duration(3.5), lambda _: self.cutscenes.put(Cutscens('bt', 4, 'sys')), oneshot=True)

        # Start scaning button state
        rospy.Timer(rospy.Duration(0.05), lambda _:self.btn.update_button())

        # control service and topic displayed
        self.set_refresh_enable_srv = rospy.Service("set_refresh_enable", SetBool, self.set_refresh_enable_srv_callback)
        self.set_on_off_srv = rospy.Service("on_off", SetBool, self.set_on_off_srv_callback)
        self.timestamp = time.time()
        rospy.on_shutdown(self.display_off)

        # Let the system status and wifi status refresh regularly
        rospy.Timer(rospy.Duration(1), self.sys_states_update)
        rospy.Timer(rospy.Duration(2), self.wireless_states_update)
        # Start to refresh the screen periodically
        rospy.Timer(rospy.Duration(0.06666), self.refresh)


    def clicked_callback(self, _):
        if self.timestamp > time.time():
            return
        self.timestamp = time.time() + 0.5
        if self.current_gram_name == 'sys':
            self.cutscenes.put_nowait(Cutscens('bt', 5, 'wireless'))
        if self.current_gram_name == 'wireless':
            self.cutscenes.put_nowait(Cutscens('bt', 5, 'sys'))

    def voltage_update(self, msg: Float32):
        self.voltage = msg.data

    def set_on_off_srv_callback(self, msg:SetBoolRequest):
        """
        设置是否开启显示
        """
        with self.lock:
            self.on_off = msg.data 
            if not self.on_off: # Set to off, that is, no display. Display a pure black screen
                self.screen.image(Image.fromarray(self.grams['clear']).convert('1'))
                self.screen.display()
            else:
                # Turn on the display and re-output the last displayed picture to the screen
                try:
                    self.screen.image(Image.fromarray(self.last_gram).convert('1'))
                    self.screen.display()
                    self.last_gram = None
                except Exception as e:
                    rospy.logerr(str(e))
        return SetBoolResponse(success=True)
    
    def display_off(self):
        self.screen.image(Image.fromarray(self.grams['clear']).convert('1'))
        self.screen.display()

    def set_refresh_enable_srv_callback(self, msg: SetBoolRequest):
        """
        设置是否刷新屏幕
        有时候会禁用刷新,例如自平衡时刷新屏幕可能会导致读取imu延时使自平衡卡顿
        """
        self.refresh_enable = msg.data
        return SetBoolResponse(success=True)
        
    def load_logo(self):
        """
        加载logo图片并且转换未可以在oled上显示的缓存数据
        """
        logo = rospy.get_param('~logo', os.path.join(rospkg.RosPack().get_path('jethexa_peripherals'), 'resources/hiwonder_logo.png'))
        gray_logo = Image.open(logo).resize((120, 14)).convert('L')
        logo_gram = np.array(gray_logo, dtype=np.uint8) // 128 * 255
        bg_gram = np.zeros((32, 128), dtype=np.uint8)
        bg_gram[9:9 + logo_gram.shape[0], 4: 4 + logo_gram.shape[1]] = logo_gram
        self.grams['logo'] = bg_gram
    
    def sys_states_update(self, timer_event):
        """
        更新系统状态信息并生成新的系统状态信息oled显示缓存
        """
        img = Image.new('1', (self.screen.width, self.screen.height))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, -1, self.screen.width, self.screen.height + 1), outline=0, fill=0)
        mem = psutil.virtual_memory()
        cpu = psutil.cpu_percent()
        disk = psutil.disk_usage('/')
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((27, 0),   "SYSTEM STATE", font=self.font, fill=0)
        draw.text((4, 11),   "CPU:{}%".format(cpu), font=self.font, fill=255)
        draw.text((66, 11),  "MEM:{:0.1f}%".format(mem.used/mem.total * 100.0), font=self.font, fill=255)
        draw.text((4, 21),  "DISK:{}%".format(int(disk.percent)), font=self.font, fill=255)
        draw.text((66, 21), "BAT:{:.2f}v".format(self.voltage), font=self.font, fill=255)
        self.grams['sys'] = np.array(img, dtype=np.uint8) * 255


    def wireless_states_update(self, timer_event):
        """
        更新WIFI状态信息并生成新的WIFI状态信息oled显示缓存
        """
        img = Image.new('1', (self.screen.width, self.screen.height))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, -1, self.screen.width, self.screen.height + 1), outline=0, fill=0)
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        wlan_ip = psutil.net_if_addrs()[self.wifi_iface][0].address
        wlan_state = dev_state('wlan0')
        draw.text((40, 0),   "WIRELESS", font=self.font, fill=0)
        draw.text((4, 11),   wlan_state['mode'] + ' SSID:' + wlan_state['ssid'], font=self.font, fill=255)
        draw.text((4, 21),   "IP:" + wlan_ip, font=self.font, fill=255)
        self.grams['wireless'] = np.array(img, dtype=np.uint8) * 255


    def imu_state_update(self, msg):
        """
        更新IMU状态信息并生成新的IMU状态信息oled显示缓存
        这个是被IMU相关包发布的topic触发的
        """
        img = Image.new('1', (self.screen.width, self.screen.height))
        draw.rectangle((0, -1, self.screen.width, self.screen.height + 1), outline=0, fill=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((50, 0),   "IMU RAW", font=self.font, fill=0)
        draw.text((4, 11), 'AX:{:1.1f} AY:{:1.1f} AZ:{:1.1f}'.format(0.0, 0.0, 0.0), font=self.font, fill=255)
        draw.text((4, 21), 'GX:{:1.1f} GY:{:1.1f} GZ:{:1.1f}'.format(0.0, 0.0, 0.0), font=self.font, fill=255)
        self.grams['imu_raw'] = np.array(img, dtype=np.uint8) * 255

        img = Image.new('1', (self.screen.width, self.screen.height))
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((38, 0),   "IMU EULER", font=self.font, fill=0)
        draw.text((4, 11), 'EX:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((64, 11), 'EX:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((4, 21), 'EZ:{:.2f} '.format(0.0), font=self.font, fill=255)
        self.grams['imu_euler'] = np.array(img, dtype=np.uint8) * 255

        draw.rectangle((0, 0, self.screen.width - 1, 11), outline=0, fill=255)
        draw.text((20, 0),   "IMU QUARTERNION", font=self.font, fill=0)
        draw.text((4, 11), 'EX:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((64, 11), 'EY:{:.2f}'.format(0.0), font=self.font, fill=255)
        draw.text((4, 21), 'EZ:{:.2f} '.format(0.0), font=self.font, fill=255)
        draw.text((64, 21), 'EW:{:.2f} '.format(0.0), font=self.font, fill=255)
        self.grams['imu_quart'] = np.array(img, dtype=np.uint8) * 255

    def refresh(self, timer_event):
        self.fps.update()
        # If the screen is not turned on or refresh is not turned on, no subsequent operations are required.
        if not self.on_off or not self.refresh_enable:
            return

        final_gram = None
        if self.current_cutscenes is None: # If the cutscene is empty then check to see if there is a cutscene to show
            try: 
                self.current_cutscenes = self.cutscenes.get(block=False)
            except Exception as e:
                pass
            if self.current_cutscenes is not None:
                self.current_cutscenes.now_gram = self.grams[self.current_gram_name]
                self.current_cutscenes.new_gram = self.grams[self.current_cutscenes.new_gram_name]

        if self.current_cutscenes is not None: # there is a cutscene
            # fly into from left to right
            if self.current_cutscenes.direction == 'lr': 
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 128:
                    now = self.current_cutscenes.now_gram[:, :-self.current_cutscenes.pixel_index]
                    new = self.current_cutscenes.new_gram[:, -self.current_cutscenes.pixel_index:]
                    final_gram = np.hstack((new, now))
                else: # If it exceeds the screen width, it means that the fly-in has been completed.
                    self.current_gram_name = self.current_cutscenes.new_gram_name # Make the new screen name the current name
                    self.current_cutscenes = None # clear cutscene

            # Fly in from right to left
            elif self.current_cutscenes.direction == 'rl':
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 128:
                    now = self.current_cutscenes.now_gram[:, self.current_cutscenes.pixel_index:]
                    new = self.current_cutscenes.new_gram[:, :self.current_cutscenes.pixel_index ]
                    final_gram = np.hstack((now, new))
                else: # If it exceeds the screen width, it means that the fly-in has been completed
                    self.current_gram_name = self.current_cutscenes.new_gram_name # Make the new screen name the current name
                    self.current_cutscenes = None # clear cutscene
            # fly into from top to buttom
            elif self.current_cutscenes.direction == 'tb': 
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 32:
                    now = self.current_cutscenes.now_gram[:-self.current_cutscenes.pixel_index, :]
                    new = self.current_cutscenes.new_gram[-self.current_cutscenes.pixel_index:, :]
                    final_gram = np.vstack((new, now))
                else: # If it exceeds the screen width, it means that the fly-in has been completed.
                    self.current_gram_name = self.current_cutscenes.new_gram_name # Make the new screen name the current name
                    self.current_cutscenes = None # clear cutscene

            # fly into from bottom to top
            elif self.current_cutscenes.direction == 'bt':
                self.current_cutscenes.pixel_index += self.current_cutscenes.increment
                if self.current_cutscenes.pixel_index <= 32:
                    now = self.current_cutscenes.now_gram[self.current_cutscenes.pixel_index:, :]
                    new = self.current_cutscenes.new_gram[:self.current_cutscenes.pixel_index, :]
                    final_gram = np.vstack((now, new))
                else: #  If it exceeds the screen width, it means that the fly-in has been completed
                    self.current_gram_name = self.current_cutscenes.new_gram_name # Make the new screen name the current name
                    self.current_cutscenes = None # clear cutscene

            else: # A transition is required, but the new screen is displayed without specifying a cutscene
                final_gram = self.current_cutscenes.new_gram
                self.current_gram_name = self.current_cutscenes.new_gram_name
                self.current_cutscenes = None
        
        if final_gram is None: #If the above cutscene does not process a new picture
            """
            有要显示的gram且与上次刷新出来的不是同一个gram才将数据输出到oled屏幕
            注意判断的是内存地址. 若用[]索引修改,因数组在内存上的地址没有改变, 不会进行刷新
            必须要替换整个数组才会刷新
            """
            if self.current_gram_name in self.grams and not self.last_gram is self.grams[self.current_gram_name]:
                final_gram = self.grams[self.current_gram_name]
            
        if final_gram is not None :
            # Invert the color of the screen display (black and white inversion) every x seconds to protect the OLED display from burning the screen
            # invert_interval is the interval of the inversion
            if self.invert_interval > 0 and time.time() - self.invert_timestamp > self.invert_interval:
                self.invert_timestamp = time.time()
                self.invert = not self.invert 
            r_final_gram = final_gram
            if self.invert:
                r_final_gram = (final_gram + 1) * 255 # The type of the factor array is uint8, so white=255, black=0, 255 + 1 = 0, 0+1 = 1, and then *255 to achieve black and white inversion

            self.screen.image(Image.fromarray(r_final_gram).convert('1'))
            try:
                self.screen.display()
                self.last_gram = r_final_gram 
            except Exception as e:
                pass
    
def main(args=None):
    try:
        oled_display_node = OledDisplayNode('oled_display_node', log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))


if __name__ == "__main__":
    main()
