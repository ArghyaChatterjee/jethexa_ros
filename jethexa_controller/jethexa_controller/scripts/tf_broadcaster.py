#!/usr/bin/env python2
# coding: utf-8

import rospy
import tf2_ros
import geometry_msgs.msg 

class jethexaOdomNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.node_name = node_name

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber('middle_tf', geometry_msgs.msg.TransformStamped, lambda tf: self.tf_broadcaster.sendTransform(tf))

def main():
    try:
        odom_publisher_node = jethexaOdomNode('mmx_jethexa_odom')
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == '__main__':
    main()
