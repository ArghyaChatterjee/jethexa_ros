#!/usr/bin/env python3
# encoding: utf-8
# @aiden
# @2022/07/28

import cv2
import rospy
import tf2_ros
from math import *
import numpy as np
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import PolygonStamped, Point32

rect_width_half = 150
rect_height_half = 130

def qua2rpy(x, y, z, w):
    roll = degrees(atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = degrees(asin(2 * (w * y - x * z)))
    yaw = degrees(atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y)))
    
    return roll, pitch, yaw

def rotate(ps, m):
    pts = np.float32(ps).reshape([-1, 2])  # the point to map
    pts = np.hstack([pts, np.ones([len(pts), 1])]).T
    target_point = np.dot(m, pts).astype(np.int)
    target_point = [[target_point[0][x], target_point[1][x]] for x in range(len(target_point[0]))]
    
    return target_point

def rotate_point(center_point, corners, angle):
    '''
    获取一组点绕一点旋转后的位置
    :param center_point:
    :param corners:
    :param angle:
    :return:
    '''
    # points [[x1, y1], [x2, y2]...]
    # 角度
    M = cv2.getRotationMatrix2D((center_point[0], center_point[1]), angle, 1)
    out_points = rotate(corners, M)

    return out_points

if __name__ == '__main__':
    rospy.init_node('virtual_wall')
    base_frame = rospy.get_param('~base_frame', 'jethexa/base_link')   
    map_frame = rospy.get_param('~map_frame', 'jethexa/map')
    delete_publish = rospy.Publisher('delete_wall', Int32, queue_size=1)
    add_publish = rospy.Publisher('add_wall', PolygonStamped, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(map_frame, base_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        x = int(trans.transform.translation.x*1000)
        y = int(trans.transform.translation.y*1000)
        rpy = qua2rpy(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
        left_up = [x - rect_width_half, y - rect_height_half]
        right_up = [x + rect_width_half, y - rect_height_half]
        left_down = [x - rect_width_half, y + rect_height_half]
        right_down = [x + rect_width_half, y + rect_height_half]
        
        out_points = rotate_point([x, y], [left_up, right_up, right_down, left_down], -rpy[-1])
        
        pose = PolygonStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = rospy.Time.now()
        points = []
        for i in out_points:
            point = Point32()
            point.x = i[0]/1000.0
            point.y = i[1]/1000.0
            point.z = 0
            points.append(point)
        pose.polygon.points = points
        add_publish.publish(pose)
        
        rate.sleep()
