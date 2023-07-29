#!/usr/bin/env python3
# coding=utf8
# 语音控制

import sys
import tts
import asr
import rospy
import math
from jethexa_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = client.Client(self)
        try:
            self.asr = asr.ASR()
            self.tts = tts.TTS()

            debug = True
            if debug:
                self.asr.eraseWords()
                self.asr.setMode(2)
                self.asr.addWords(1, 'kai shi')
                self.asr.addWords(2, 'wang qian zou')
                self.asr.addWords(2, 'qian jin')
                self.asr.addWords(2, 'zhi zou')
                self.asr.addWords(3, 'wang hou tui')
                self.asr.addWords(4, 'xiang zuo yi dong')
                self.asr.addWords(5, 'xiang you yi dong')

            data = self.asr.getResult()
            self.tts.TTSModuleSpeak('[h0][v10][m3]', '准备就绪')
            print('''
口令：开始
指令2：往前走
指令2：前进
指令2：直走
指令3：往后退
指令4：向左移动
指令5：向右移动
''')
            rospy.sleep(2)
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr('传感器初始化出错')
        rospy.Timer(rospy.Duration(0.1), self.asr_callback, oneshot=True)

    def asr_callback(self, t):
        data = self.asr.getResult()
        if data != 0:
            rospy.loginfo(data)
            self.tts.TTSModuleSpeak('', '收到')
            rospy.sleep(1)
            if data == 2:
                self.jethexa.traveling(
                        gait=1,
                        stride=30.0,
                        height=25.0, # 步高 25mm
                        direction=0, # 180 方向移动就是后移
                        rotation=0.0,
                        time=1.0,
                        steps=5)
            elif data == 3:
                self.jethexa.traveling(
                        gait=1,
                        stride=30.0,
                        height=25.0, # 步高 25mm
                        direction=math.radians(180), # 180 方向移动就是后移
                        rotation=0.0,
                        time=1.0,
                        steps=5)
            elif data == 4:
                self.jethexa.traveling(
                        gait=1,
                        stride=30.0,
                        height=25.0, # 步高 25mm
                        direction=math.radians(90), # 180 方向移动就是后移
                        rotation=0.0,
                        time=1.0,
                        steps=5)
            elif data == 5:
                self.jethexa.traveling(
                        gait=1,
                        stride=30.0,
                        height=25.0, # 步高 25mm
                        direction=math.radians(270), # 180 方向移动就是后移
                        rotation=0.0,
                        time=1.0,
                        steps=5)
        rospy.Timer(rospy.Duration(0.02), self.asr_callback, oneshot=True)

    def start(self):
        self.jethexa.traveling(
                  gait=1, # RIPPER步态
                  stride=0.0, # 步幅 60mm
                  height=25.0, # 步高 25mm
                  direction=0, # 180 方向移动就是后移
                  rotation=0.0,
                  time=1.2, # 每步的用时
                  steps=0, # 走多少步, 0步就是一直走，直到被新的指令改变
                  interrupt=True,
                  relative_height=False)
    
    def stop(self):
        rospy.loginfo("stop")
        self.jethexa.traveling(gait=-2)
    

if __name__ == "__main__":
    node = MovingNode() # 建立相关资源
    rospy.sleep(3) # 稍等一下下, 订阅或者注册发布之后可能不会马上生效要等一下下
    rospy.on_shutdown(node.stop) # 注册退出时的回调， 退出时停止机器人
    rospy.spin() # 等待退出, spin() 会保持进行的活跃但是并不做什么事

