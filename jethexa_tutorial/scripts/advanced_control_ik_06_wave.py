#!/usr/bin/env python3

import sys
import argparse
import rospy
import math
from jethexa_controller import build_in_pose, kinematics_api, jethexa

class Control:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = jethexa.JetHexa(self)
    
    def wave(self):
        """
        机体扭动
        """
        duration = 0.03
        self.jethexa.set_pose_base(build_in_pose.DEFAULT_POSE_M, 0.8)
        org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
        rospy.sleep(0.8)
        # Gradually speed up and increase the swing
        for j in range(7, 20, 2):
            i = 90 
            j = min(15, j)
            while i <= 360 + 85:
                if i == 90 and j == 7:
                    t = 0.5
                else:
                    t = duration
                i += 4 + j * 0.30
                x = math.sin(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                y = math.cos(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                pose = kinematics_api.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.jethexa.set_pose_base(pose, t)
                rospy.sleep(t)

        # Gradually speed down and decrease the swing
        for j in range(15, 4, -3):
            i = 360 + 85
            while i >= 90:
                i += -(4 + j * 0.30)
                k = 360 + 90 - i + 90
                x = math.sin(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                y = math.cos(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                pose = kinematics_api.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.jethexa.set_pose_base(pose, duration)
                rospy.sleep(duration)
        # restore to normal

    def reset(self):
        self.jethexa.set_pose_base(build_in_pose.DEFAULT_POSE_M, 1)

if __name__ == "__main__":
    argv = rospy.myargv(argv=sys.argv) # ros will pass in some of its own parameters when starting the py file, and myargv will remove these and return the original parameters
    parser = argparse.ArgumentParser() # parameter parser
    parser.add_argument('direction', metavar='direction', nargs='?', type=float, help="指定平移方向", default=0) # Add parameters to parse
    argv = parser.parse_args(argv[1:]) # parse the input parameter

    node = Control() # create related resources
    rospy.loginfo("start")
    rospy.loginfo("move direction: {:0.2f}°".format(argv.direction))
    rospy.on_shutdown(node.reset)
    rospy.sleep(3)
    node.wave() 


