#!/usr/bin/env python3

import argparse
import sys
import rospy
from jethexa_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = client.Client(self)
    
    def start(self, period, stride):
        self.jethexa.traveling(
                  gait=1, # RIPPER gait
                  stride=stride, # stride
                  height=15.0, # step height 25mm
                  direction=0, # move in the direction of 180 that is move backward
                  rotation=0.0,
                  time=period, # time taken for each step
                  steps=0, # how many steps take. 0 means the robot will keep walking till it is changed by new command.
                  interrupt=True,
                  relative_height=False)
    
    def stop(self):
        rospy.loginfo("stop")
        self.jethexa.traveling(gait=0)
    

if __name__ == "__main__":
    node = MovingNode() # create the related resources
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--period', "-p", type=float, help="每步间隔，单位: 秒", default=0.8)
    parser.add_argument('--stride', "-s", type=float, help="步幅", default=40)
    argv = parser.parse_args(argv[1:]) # Parse input parameters
    rospy.sleep(3) # Waiting for exit, and spin() will remain active but nothing will happend
    node.start(argv.period, argv.stride) # walk straight

