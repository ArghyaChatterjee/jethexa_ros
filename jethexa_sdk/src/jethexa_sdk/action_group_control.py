#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import time
import sqlite3 as sql
from .serial_servo import set_position, stop

#actions liabrary edited on PC software 

runningAction = False
stop_action = False
stop_action_group = False

def stopAction():
    global stop_action
    
    stop_action = True

def runActionGroup(actName, times=1):
    global stop_action
    global stop_action_group

    stop_action = False

    temp = times
    while True:
        if temp != 0:
            times -= 1
            if times < 0 or stop_action_group: # get out of the loop
                stop_action_group = False
                break
            runAction(actName)
        elif temp == 0:
            if stop_action_group: # get out of the loop
                stop_action_group = False
                break
            runAction(actName)

def runAction(actNum, lock_servos=''):
    '''
    运行动作组，无法发送stop停止信号
    :param actNum: 动作组名字 ， 字符串类型
    :param times:  运行次数
    :return:
    '''
    global runningAction
    global stop_action
    global stop_action_group

    if actNum is None:
        return

    actNum = "/home/hiwonder/ActionSets/" + actNum + ".d6a"

    if os.path.exists(actNum) is True:
        if runningAction is False:
            runningAction = True
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")
            while True:
                act = cu.fetchone()
                if stop_action:
                    stop_action_group = True
                    break
                if act is not None:
                    for i in range(0, len(act) - 2, 1):
                        if str(i + 1) in lock_servos:
                            set_position(i + 1, lock_servos[str(i + 1)], act[1])
                        else:
                            set_position(i + 1, act[2 + i], act[1])
                    for j in range(int(act[1]/50)):
                        if stop_action:
                            stop_action_group = True
                            break
                        time.sleep(0.05)
                    time.sleep(0.001 + act[1]/1000.0 - 0.05*int(act[1]/50))
                else:   # exit after finishing running
                    break
            runningAction = False
            
            cu.close()
            ag.close()
    else:
        runningAction = False
        print("未能找到动作组文件")
