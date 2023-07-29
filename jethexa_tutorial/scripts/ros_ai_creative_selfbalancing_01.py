#!/usr/bin/env python3

import rospy
from scipy.spatial.transform import Rotation as R
import sensor_msgs.msg

class SelfBalancingNode:
    def __init__(self):
        rospy.init_node("self_balancing_node_")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/filtered")
        rospy.loginfo(self.imu_topic)
        
        self.imu_sub = rospy.Subscriber(self.imu_topic, sensor_msgs.msg.Imu, self.imu_callback, queue_size=2)

    def imu_callback(self, imu_msg):
        try:
            q = imu_msg.orientation
            r = R.from_quat((q.x, q.y, q.z, q.w))
            x, y, z = r.as_euler('xyz', degrees=True)
            rospy.loginfo("Pitch:{:>6.2f}°, Roll:{:>6.2f}°, Yaw:{:>6.2f}°".format(y, x, z))
        except Exception as e:
            rospy.logerr(str(e))


if __name__ == "__main__":
    try:
        self_balancing_node = SelfBalancingNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
