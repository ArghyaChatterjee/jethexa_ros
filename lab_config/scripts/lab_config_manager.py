#!/usr/bin/env python3

"""
此程序提供了阈值调节,保存等功能的服务
"""
import os
import sys
import cv2
import rospy
import yaml
import numpy as np
from threading import RLock
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, Trigger, SetBool, TriggerResponse, SetBoolResponse
from lab_config.srv import StashRange, GetRange, ChangeRange, GetAllColorName
from lab_config.srv import StashRangeResponse, GetRangeResponse, ChangeRangeResponse, GetAllColorNameResponse


class LabConfigManagerNode:
    def __init__(self, node_name):
        rospy.init_node(node_name, log_level=rospy.INFO)
        self.node_name = node_name

        # Read the required parameters
        self.config_file_path = rospy.get_param('~config_file_path', os.path.join(sys.path[0], "../config/lab_config.yaml"))
        self.color_ranges = rospy.get_param('~color_range_list', {})
        self.kernel_erode = rospy.get_param('~kernel_erode', 3)
        self.kernel_dilate = rospy.get_param('~kernel_dilate', 3)
        self.current_range = {'min': [0, 0, 0], 'max': [100, 100, 100]}
        if 'red' in self.color_ranges:
            self.current_range = self.color_ranges['red']

        # topic related to the screen
        self.image_sub = None
        self.result_image_pub = rospy.Publisher(self.node_name + '/image_result', Image, queue_size=1)

        # enter, exit, start, stop service
        self.enter_srv = rospy.Service(self.node_name + '/enter', Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service(self.node_name + '/exit', Trigger, self.exit_srv_callback)
        self.running_srv = rospy.Service(self.node_name + '/set_running', SetBool, self.set_running_srv_callback)

        # Modify thresholds, and maintain thresholds and other services
        self.save_to_disk_srv = rospy.Service(self.node_name + '/save_to_disk', Trigger, self.save_to_disk_srv_callback)
        self.get_color_range_srv = rospy.Service(self.node_name + '/get_range', GetRange, self.get_range_srv_callback)
        self.change_range_srv = rospy.Service(self.node_name + '/change_range', ChangeRange, self.change_range_srv_callback)
        self.stash_range_srv = rospy.Service(self.node_name + '/stash_range', StashRange, self.stash_range_srv_callback)
        self.get_all_color_name_srv = rospy.Service(self.node_name + '/get_all_color_name', GetAllColorName,
                                           self.get_all_color_name_srv_callback)

        # Heartbeat. Ensure that the function can automatically stop running when the APP exits abnormally, to avoid resource occupation
        self.heartbeat_timer = None
        self.heartbeat_srv = rospy.Service('lab_config_manager/heartbeat', SetBool, self.heartbeat_srv_callback)

    def image_callback(self, ros_image):
        """
        相机画面回调
        :params ros_image: 画面数据
        """
        range_ = self.current_range
        # Convert ros format image to opencv format
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)

        # Image processing and binarization
        image_resize = cv2.resize(image, (320, 180), interpolation=cv2.INTER_NEAREST)
        frame_result = cv2.cvtColor(image_resize, cv2.COLOR_RGB2LAB)
        frame_result = cv2.GaussianBlur(frame_result, (3, 3), 3)
        mask = cv2.inRange(frame_result, tuple(range_['min']), tuple(range_['max']))  # Bitwise operations on the original image and mask
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_erode, self.kernel_erode)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_dilate, self.kernel_dilate)))

        # Publish the processed binarized image
        dilated = cv2.resize(dilated, (640, 360))
        rgb_image = cv2.cvtColor(dilated, cv2.COLOR_GRAY2RGB).tostring()
        ros_image.data = rgb_image
        ros_image.height = 360
        ros_image.width = 640
        ros_image.step = ros_image.width * 3
        self.result_image_pub.publish(ros_image)


    def enter_srv_callback(self, _):
        """
        APP 进入功能
        注册对相机的订阅
        """
        rospy.loginfo('enter')
        try:
            self.image_sub.unregister() # Always try to cancel the subscription to the camera to avoid resource occupation and exceptions caused by repeated subscriptions
        except Exception as e:
            rospy.logerr(str(e))
        self.camera_rgb_prefix = rospy.get_param('/camera_rgb_prefix', 'camera/rgb')
        self.image_sub = rospy.Subscriber(self.camera_rgb_prefix + '/image_raw', Image, self.image_callback, queue_size=1)
        return [True, '']
    

    def exit_srv_callback(self, _):
        """
        APP退出功能
        会注销掉相机的订阅, 并停止心跳定时器
        """
        rospy.loginfo('exit')
        try:
            self.image_sub.unregister() # 注销订阅
        except Exception as e:
            rospy.logerr(str(e))
        try:
            self.heartbeat_timer.shutdown() # 注销订阅
        except Exception as e:
            rospy.logerr(str(e))
        return [True, '']
    
    
    def set_running_srv_callback(_):
        """
        本来时用来控制运行或者暂停运行的, 但是这里废弃了
        为了保持兼容性留着这个空函数
        """
        rospy.loginfo("set running called")
        return [True, 'set_running']
    
    
    def save_to_disk_srv_callback(self, _):
        """
        保存当前的阈值列表到硬盘(sd卡)中
        """
        rospy.loginfo("saving thresholds to dist")
        # Put the list of thresholds into a dictionary of the corresponding format
        cf = {"color_range_list": self.color_ranges} 
        rospy.loginfo(cf)
        # The dictionary is converted to yaml format string and written to the file
        with open(self.config_file_path, 'w') as f:
            f.write(yaml.dump(cf, default_flow_style=False))
        return TriggerResponse(success=True)
    
    
    def get_range_srv_callback(self, msg):
        """
        获取指定颜色的阈值
        """
        rospy.loginfo(msg)
        rsp = GetRangeResponse()
        ranges = rospy.get_param('~color_range_list', self.color_ranges) # 从参数服务器获取所有颜色阈值的了列表
        if msg.color_name in ranges: # 在阈值列表中有要获取的颜色
            rsp.success = True
            # Fill the threshold into the returned result
            rsp.min = ranges[msg.color_name]['min']
            rsp.max = ranges[msg.color_name]['max']
        else:
            rsp.success = False
            color_ranges = ranges
        return rsp
    
    
    def change_range_srv_callback(self, msg):
        """
        修改当前的颜色阈值
        就是控制结果画面改变
        :param msg: msg.min 阈值下限， msg.max 阈值上限
        """
        rospy.loginfo(msg)
        self.current_range = dict(min=list(msg.min), max=list(msg.max))
        return ChangeRangeResponse(success=True)
    
    
    def stash_range_srv_callback(self, msg):
        """
        暂存当前阈值
        修改指定颜色的阈值为当前阈值(就是结果画面对应的阈值)
        :param msg: msg.color_name 要修改的颜色名称
        """
        rospy.loginfo(msg)
        ranges = rospy.get_param('~color_range_list', self.color_ranges) # 获取当前的颜色阈值列表
        ranges[msg.color_name] = self.current_range  # 修改指定颜色的阈值
        rospy.set_param('~color_range_list', ranges) # 存回参数服务器
        self.color_ranges = ranges
        return StashRangeResponse(success=True)
    
    
    def get_all_color_name_srv_callback(self, msg):
        """
        获取保存的全部颜色的名称
        """
        rospy.loginfo(msg)
        ranges = rospy.get_param('~color_range_list', self.color_ranges) # 从参数服务器获取所有颜色阈值
        color_names = list(ranges.keys()) # 取键名, 就是颜色名称
        return GetAllColorNameResponse(color_names=color_names)
    
    def heartbeat_timeout_callback(self, _):
        """
        心跳超时回调
        """
        rospy.loginfo("heartbeat timeout. exiting...")
        self.exit_srv_callback(None)  # 停止功能的运行
    
    def heartbeat_srv_callback(self, msg):
        """
        心跳服务回调
        :param msg: msg.data 控制起跳或停跳, 为True时起跳
        """
        try:
            # Stop the last timer regardless of jumping or stopping
            self.heartbeat_timer.shutdown()
        except Exception as e:
            rospy.logerr(str(e))
        rospy.logdebug("Heartbeat, " + str(msg))
        if msg.data:
            # Take off and create a new timer
            self.heartbeat_timer = rospy.Timer(rospy.Duration(5), self.heartbeat_timeout_callback, oneshot=True)
        return SetBoolResponse(success=True)
    

if __name__ == '__main__':
    try:
        lab_conf_manager_node = LabConfigManagerNode('lab_config_manager')
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        sys.exit(0)
