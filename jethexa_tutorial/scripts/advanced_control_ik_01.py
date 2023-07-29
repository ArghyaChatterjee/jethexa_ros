#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from jethexa_controller import jethexa


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = jethexa.JetHexa(self, pwm=False)
    
    def start(self):
        for i in range(3):
            joints = self.jethexa.set_leg_position(2, (0, 140, -50), 2)
            print(joints)
            rospy.sleep(2.5)
            joints = self.jethexa.set_leg_position(2, (0, 250, -50), 2)
            print(joints)
            rospy.sleep(2.5)

    
    


if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros will pass in some of its own parameters when starting the py file, and myargv will remove these and return the original parameters

    node = MovingNode() # create related resources
    rospy.sleep(3)
    node.start() # run


