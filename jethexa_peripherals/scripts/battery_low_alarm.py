#!/usr/bin/env python3

# This file is one part of JetHexa 
# battery_low_alarm.py realizes low voltage alarm

import time
import rospy
import std_msgs.msg
from jethexa_sdk import buzzer


class BatteryLowAlarmNode:
    def __init__(self, name, anonymous=True, log_level=rospy.INFO):
        rospy.init_node(name, log_level=log_level)
        self.beep = False
        self.beep_on = False
        self.timer = None
        rospy.on_shutdown(self.shutdown_callback)
        self.voltage_sub = rospy.Subscriber('voltage', std_msgs.msg.Float32, self.voltage_update, queue_size=1)

    def voltage_update(self, msg: std_msgs.msg.Float32):
        voltage = msg.data
        if voltage > 5.0 and voltage < 9.8:
            if not self.beep:
                self.beep = True
                self.timer_callback(None)

    def timer_callback(self, _):
        if self.beep:
            if self.beep_on:
                self.beep_on = False
                buzzer.off()
            else:
                self.beep_on = True
                buzzer.on()
            self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback, oneshot=True)
        else:
            buzzer.off()
            

    def shutdown_callback(self):
        self.beep = False
        try:
            self.timer.shutdown()
        except Exception as e:
            pass
        buzzer.off()


if __name__ == "__main__":
    try:
        battery_low_alarm_node = BatteryLowAlarmNode('battery_low_alarm_node', log_level=rospy.INFO)
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

