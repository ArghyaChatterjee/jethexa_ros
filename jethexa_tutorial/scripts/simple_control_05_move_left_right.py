#!/usr/bin/env python3

import argparse
import sys
import rospy
import math
from jethexa_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = client.Client(self)
    
    def start(self, d):
        if d == 'left':
            d = math.radians(90)
        elif d == 'right':
            d = math.radians(270)
        else:
            sys.exit(-1)
        self.jethexa.traveling(
                  gait=1, # RIPPER gait 
                  stride=40.0, # stride 40mm
                  height=25.0, # step height 25mm
                  direction=d, # counterclockwise is the positive direction. 90 degree refers to left side. And 270 degree refers to right side.
                  rotation=0.0,
                  time=1.2, # Time taken for each step
                  steps=0, # how many steps take. 0 means the robot will keep walking till it is changed by the new command
                  interrupt=True,
                  relative_height=False)
    
    def stop(self):
        rospy.loginfo("stop")
        self.jethexa.traveling(gait=0)
    


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros will pass in some of its own parameters when starting the py file, and myargv will remove these and return the original parameters
    parser = argparse.ArgumentParser() # parameter parser
    parser.add_argument('left_or_right', metavar='left or right', nargs='?', type=str, help="左移或右移", default="left") # Add parameters to parse
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    node = MovingNode() # Create relevant resources
    rospy.sleep(3) # Please wait a moment, because the subscription or registration may not take effect immediately after the release
    rospy.loginfo("start")
    node.start(argv.left_or_right) # walk straight
    rospy.on_shutdown(node.stop) # callback when exiting the registration. Stop the robot when exiting.
    rospy.spin() # Waiting for exit, and spin() will remain active but nothing will happend

