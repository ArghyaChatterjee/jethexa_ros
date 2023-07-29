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
    
    def start(self, direction):
        self.jethexa.traveling(
                  gait=1, # RIPPER gait 
                  stride=40.0, # stride 40mm
                  height=25.0, # gait 25mm
                  direction=math.radians(direction), 
                  rotation=0.0,
                  time=1.2, # Time taken for each step
                  steps=0, # how many steps take. 0 means the robot will keep walking till it is changed by the new command
                  interrupt=True,
                  relative_height=False)
    
    def stop(self):
        rospy.loginfo("stop")
        self.jethexa.traveling(gait=0)
    


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) #  ros will pass in some of its own parameters when starting the py file, and myargv will remove these and return the original parameters
    parser = argparse.ArgumentParser() # parameter parser
    parser.add_argument('direction', metavar='direction', nargs='?', type=float, help="指定平移方向", default=0) # Add parameters to parse
    argv = parser.parse_args(argv[1:]) # Parse input parameters

    node = MovingNode() # Create relevant resources
    rospy.sleep(3) # Please wait a moment, because the subscription or registration may not take effect immediately after the release
    rospy.loginfo("start")
    rospy.loginfo("move direction: {:0.2f}°".format(argv.direction))

    node.start(argv.direction) # move in the designated direction
    rospy.on_shutdown(node.stop) # callback when exiting the registration. Stop the robot when exiting.
    rospy.spin() # Waiting for exit, and spin() will remain active but nothing will happend


