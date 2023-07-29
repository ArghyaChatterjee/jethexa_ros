#!/usr/bin/env python3

import rospy
from jethexa_controller import jethexa

class Control:
    def __init__(self):
        rospy.init_node("moving_node", anonymous=True, log_level=rospy.INFO)
        self.jethexa = jethexa.JetHexa(self)
        self.jethexa.set_build_in_pose('DEFAULT_POSE', 1)
        rospy.sleep(1)
        self.jethexa.set_step_mode(1, 40, 15, 0, 0, 0.8, repeat=0) # robot walks straight
    
    def transform(self):
        for _ in range(10):
            for i in range(20):
                self.jethexa.transform_pose_2((0, 0, 3), 'xyz', (0,0,0), 0.1) # raise the robot
                rospy.sleep(0.1)
            for i in range(20):
                self.jethexa.transform_pose_2((0, 0, -3), 'xyz', (0,0,0), 0.1) # lower the robot
                rospy.sleep(0.1)

    def reset(self):
        self.jethexa.set_build_in_pose('DEFAULT_POSE', 1)
        rospy.sleep(1)
    
if __name__ == "__main__":
    node = Control() # create related resource
    rospy.loginfo("start")
    rospy.on_shutdown(node.reset) # reset when exiting
    rospy.sleep(3)
    try:
        node.transform()
    except Exception as e:
        rospy.logerr(str(e))


