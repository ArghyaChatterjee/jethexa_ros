#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from jethexa_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = client.Client(self)
    
    def start(self):
        a = [45, 360 - 45]
        while True:
            for i in a:
                self.jethexa.traveling(
                  gait=1, # RIPPER gait 
                  stride=40.0, # stride 40mm
                  height=20.0, # step height 20mm
                  direction=math.radians(i), 
                  rotation=0.0,
                  time=0.8, # time taken for each step
                  steps=0, # how many steps it takes. 0 means it will keep walking till the refreshed command is changed
                  interrupt=True,
                  relative_height=False)
                rospy.sleep(4)
    
    def stop(self):
        rospy.loginfo("stop")
        self.jethexa.traveling(gait=-2)
    


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros will pass in some of its own parameters when starting the py file, myargv will remove these and return the original parameters
    parser = argparse.ArgumentParser() # parameter parser
    # parser.add_argument('direction', metavar='direction', nargs='?', type=float, help="指定平移方向", default=0) # Add parameters to parse
    # argv = parser.parse_args(argv[1:]) # Parse input parameters

    node = MovingNode() # create the related resources
    rospy.sleep(3) # Please wait a moment, because the subscription or registration may not take effect immediately after the release.
    rospy.loginfo("start")

    rospy.on_shutdown(node.stop) # callback when exitting registering. And the robot will stop when exiting
    node.start() # move in the designated direction


