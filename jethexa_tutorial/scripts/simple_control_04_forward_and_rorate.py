#!/usr/bin/env python3

import rospy
from jethexa_controller import client


class MovingNode:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = client.Client(self)
    
    def forward(self):
        self.jethexa.traveling(
                  gait=1, # RIPPER gait
                  stride=40.0, # stride 40mm
                  height=15.0, # step height 15mm
                  direction=0, # move in the direction of 180 that is move backward
                  rotation=0.0,
                  time=1, # Time taken for each step
                  steps=0, # how many steps take. 0 means the robot will keep walking till it is changed by the new command
                  interrupt=True,
                  relative_height=False)
    def rotate(self):
        self.jethexa.traveling(
                  gait=1, # RIPPER gait
                  stride=40.0, # stride 40mm
                  height=15.0, # step height 15mm
                  direction=0, # move in the direction of 180 that is move backward
                  rotation=0.5,
                  time=0.8, # Time taken for each step
                  steps=0, # how many steps take. 0 means the robot will keep walking till it is changed by the new command
                  interrupt=True,
                  relative_height=False)
    def stop(self):
        rospy.loginfo("stop")
        self.jethexa.traveling(gait=0)
    

if __name__ == "__main__":
    node = MovingNode() # create the related resources
    rospy.sleep(3) # Please wait a moment, because the subscription or registration may not take effect immediately after the release
    node.forward() # walk straight
    rospy.sleep(10)
    node.rotate()   # turn
    rospy.sleep(10)
    node.stop() #stop
    rospy.on_shutdown(node.stop) # callback when exiting the registration. Stop the robot when exiting.
    rospy.spin() # Waiting for exit, and spin() will remain active but nothing will happend

