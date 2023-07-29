#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from jethexa_controller import jethexa

class Control:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = jethexa.JetHexa(self)
        self.jethexa.set_build_in_pose('DEFAULT_POSE_M', 1)
        rospy.sleep(1.0)
    
    def transform(self, translation, axis, rotation, duration):
        x, y, z = translation
        u, v, w = rotation
        self.jethexa.transform_pose_2((x, y, z), axis, (u,v,w), duration, degrees=True)

    def do(self):
        self.transform((0, 0, 0), 'xyz', (-15, 0, 0), 1) # rotation
        rospy.sleep(1)
        self.transform((0, 0, 0), 'xyz', (0, 15, 0), 1) # pay attention to the order of Euler angle
        rospy.sleep(1)
        self.transform((0, 0, 0), 'yxz', (-15, 15, 0), 2)  #pay attention to the order of Euler angle
        rospy.sleep(2)
        self.transform((20, 0, 0), 'xyz', (0, 0, 0), 1) # translation in mm
        rospy.sleep(1)
        self.transform((0, 20, 0), 'xyz', (0, 0, 0), 1)
        rospy.sleep(1)
        self.transform((-20, -20, 0), 'xyz', (0, 0, 0), 1)
        rospy.sleep(1)
    
    def reset(self):
        self.jethexa.set_build_in_pose('DEFAULT_POSE_M', 1)
    
if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros will pass in some of its own parameters when starting the py file, myargv will remove these and return the original parameters

    node = Control() # create related files
    rospy.loginfo("start")
    rospy.on_shutdown(node.reset)
    rospy.sleep(3)
    node.do() 


