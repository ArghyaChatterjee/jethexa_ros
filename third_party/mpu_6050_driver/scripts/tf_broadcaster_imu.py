#!/usr/bin/env python2 

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu

def handle_imu_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = target_frame
    t.child_frame_id = imu_link
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_imu_2')
    imu_link = rospy.get_param('~imu_link', 'imu_link')
    target_frame = rospy.get_param('~fixed_frame', 'world')
    imu_topic = rospy.get_param('~imu_topic', "/imu/filtered")
    rospy.Subscriber(imu_topic, Imu, handle_imu_pose)
    rospy.spin()
