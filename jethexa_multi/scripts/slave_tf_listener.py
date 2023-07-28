#!/usr/bin/env python
# encoding: utf-8
import tf
import math
import rospy
import jethexa_sdk.pid as pid
from geometry_msgs.msg import Twist

def qua2rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener() #After the TransformListener is created, it starts to accept tf broadcast information, which can be cached for up to 10s

    cmd_vel = rospy.get_param('~cmd_vel', '/jethexa_1/jethexa_controller/cmd_vel')
    base_frame = rospy.get_param('~base_frame', '/jethexa_1/base_link')   
    target_frame = rospy.get_param('~target_frame', '/row_1')
    
    robot_vel = rospy.Publisher(cmd_vel, Twist, queue_size=1)
    rate = rospy.Rate(30.0) #excute in loop, and the update frequency is 10hz
    
    pid_x = pid.PID(0.4, 0.0, 0.05)
    pid_y = pid.PID(0.4, 0.0, 0.05)
    pid_z = pid.PID(0.013, 0.0, 0.002)
    
    while not rospy.is_shutdown():
        msg = Twist()
        try:
            (trans, rot) = listener.lookupTransform(base_frame, target_frame, rospy.Time()) #check the relative tf, and return translation and rotation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
            robot_vel.publish(msg)
            rate.sleep()
            continue
        
        #print(trans, rot)
         
        x = trans[0]
        y = trans[1]
        angle = math.degrees(qua2rpy(rot[0], rot[1], rot[2], rot[3])[-1])
          
        pid_x.SetPoint = 0
        if abs(x) < 0.01:
            x = 0
        pid_x.update(x)  #update pid
        linear_x = -pid_x.output
       
        pid_y.SetPoint = 0
        if abs(y) < 0.01:
            y = 0
        pid_y.update(y)  #update pid
        linear_y = -pid_y.output
        
        pid_z.SetPoint = 0
        if abs(angle) < 2:
            angle = 0
        pid_z.update(angle)  #update pid
        angular_z = -pid_z.output

        linear_x = min(max(linear_x, -0.1), 0.1)
        linear_y = min(max(linear_y, -0.06), 0.06)
        angular_z = min(max(angular_z, -0.25), 0.25)
        
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        
        robot_vel.publish(msg)
        rate.sleep() #execute in fixed frequency
