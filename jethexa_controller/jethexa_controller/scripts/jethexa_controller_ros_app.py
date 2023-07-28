#!/usr/bin/env python3
# coding: utf-8

import time
import rospy
import nav_msgs.msg as nav_msgs
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped, TwistWithCovarianceStamped
#
from jethexa_controller_interfaces import msg as jetmsg
from jethexa_controller_interfaces.srv import SetPose1, SetPose1Request, SetPose1Response
from jethexa_controller_interfaces.srv import SetPose2, SetPose2Request, SetPose2Response
from jethexa_controller_interfaces.srv import PoseTransform, PoseTransformRequest, PoseTransformResponse
#
import jethexa_sdk.buzzer as buzzer
from jethexa_controller import jethexa, build_in_pose, config
from jethexa_controller.z_voltage_publisher import VoltagePublisher
from jethexa_controller.z_joint_states_publisher import JointStatesPublisher
import geometry_msgs.msg


class jethexaControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        self.tf_prefix = rospy.get_param('~tf_prefix', '')
        self.tf_prefix = (self.tf_prefix + '/') if self.tf_prefix != '' else '' 

        self.controller = jethexa.JetHexa(self)

        # publish the status of the robot
        self.voltage_publisher = VoltagePublisher(node=self, rate=1)  # publish the bus voltage
        self.joint_states_publisher = JointStatesPublisher(node=self, rate=20)  # publish joint angle

        self.controller.set_build_in_pose('DEFAULT_POSE', 2)

        # robot posture setting service
        self.set_pose1_srv = rospy.Service("~set_pose_1", SetPose1, self.set_pose1_cb) # set the robot posture through the posture name, "DEFAULT_POSE" "DEFAULT_POSE_M"
        self.set_pose2_srv = rospy.Service("~set_pose_2", SetPose2, self.set_pose2_cb) # set the robot posture through six foothold coordinates 

        # transform the robot posture
        # transform the robot posture through translation and Euler angle rotation. It belongs to relative transformation, and the rotation order is RPY
        self.set_transform2_sub = rospy.Subscriber("~pose_transform_euler", jetmsg.TransformEuler, self.pose_transform_euler_cb)
        # transform the robot posture through translation and Euler angle rotation. It belongs to absolute transformation, and the rotation order is RPY
        self.set_pose_euler_sub = rospy.Subscriber("~set_pose_euler", jetmsg.Pose, self.set_pose_euler_cb)
        # set the specific position to which the robot's foot rotates to. It belongs to absolute coordinate.
        self.set_leg_position_sub = rospy.Subscriber("~set_leg_absolute", jetmsg.LegPosition, self.set_leg_absolute_cb)
        # Set a foot to move to the specified position relative to the current position
        self.set_leg_position_re_sub = rospy.Subscriber("~set_leg_relatively", jetmsg.LegPosition, self.set_leg_relatively_cb)

        # robot head posture control
        # absolute rotation. When RPY=0, 0, 0, the pan-tilt will face to the front 
        self.set_head_absolute = rospy.Subscriber("~set_head_absolute", jetmsg.TransformEuler, self.head_absolute_cb)
        # relative rotation
        self.set_head_relatively = rospy.Subscriber("~set_head_relatively", jetmsg.TransformEuler, self.head_relatively_cb)

        # robot movement service
        # control the robot to move through gait parameter
        self.traveling = rospy.Subscriber("~traveling", jetmsg.Traveling, self.set_traveling_cb)
        # control robot's movement through linear velocity and angular velocity. Other parameters are specified by traveling with the last executed gait greater than 0
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, self.cmd_vel_callback)

        # action group running service
        self.run_action_sub = rospy.Subscriber("~run_actionset", jetmsg.RunActionSet, self.run_action_set_sub_cb)

        # Subscribe to the precise odom after integrating imu and lidar to obtain accurate yaw angle to improve odometer accuracy
        self.odom_sub = rospy.Subscriber("odom/filtered", nav_msgs.Odometry, self.odom_callback)

        # publish odom regularly
        odom_enable = rospy.get_param('~odom_enable', False)
        if odom_enable:
            # publish related odom data
            self.last_time_odometry = time.time()
            # publish data to python2 to publish tf transformation
            self.odom_trans_pub = rospy.Publisher("~middle_tf", TransformStamped, queue_size=2) 
            # original gait odom
            self.odometry_pub = rospy.Publisher("odom/raw", nav_msgs.Odometry, queue_size=2) 
            # original speed
            self.twist_pub = rospy.Publisher("twist_raw", TwistWithCovarianceStamped, queue_size=2) 
            self.odom_timer = rospy.Timer(rospy.Duration(0.02), self.odometry_publish)

        # end
        buzzer.on()
        time.sleep(0.1)
        buzzer.off()

    def cmd_vel_callback(self, msg:geometry_msgs.msg.Twist):
        lx = msg.linear.x * 0.1
        ly = msg.linear.y * 0.1
        az = min(max(msg.angular.z, -0.25), 0.25)
        msg.linear.x = lx
        msg.linear.y = ly
        msg.angular.z = az
        self.controller.cmd_vel(msg)
    
    def odom_callback(self, msg: nav_msgs.Odometry):
        o = msg.pose.pose.orientation
        r = R.from_quat((o.x, o.y, o.z, o.w))
        yaw = r.as_euler('xyz', degrees=False)
        self.controller.real_pose_yaw = yaw[-1]


    def head_absolute_cb(self, msg: jetmsg.TransformEuler):
        yaw = msg.rotation.z
        pitch = msg.rotation.y
        rospy.loginfo("Set head absolutly pitch:{} yaw:{}".format(pitch, yaw))
        try:
            self.controller.set_joint(19, yaw, msg.duration)
            self.controller.set_joint(20, pitch, msg.duration)
        except Exception as e:
            rospy.logerr(str(e))

    def head_relatively_cb(self, msg: jetmsg.TransformEuler):
        yaw = msg.rotation.z
        pitch = msg.rotation.y
        old_yaw = self.controller.joints_state['head_pan_joint']  # current joint angle
        old_pitch = self.controller.joints_state['head_tilt_joint']
        new_yaw = old_yaw + yaw
        new_pitch = old_pitch + pitch
        if new_pitch < -0.35:
            return
        try:
            self.controller.set_joint(19, new_yaw, msg.duration)
            self.controller.set_joint(20, new_pitch, msg.duration)
        except Exception as e:
            rospy.logerr(str(e))

    def set_traveling_cb(self, msg: jetmsg.Traveling):
        gait = msg.gait
        height = msg.height
        stride = msg.stride
        direction = msg.direction
        rotation = msg.rotation
        steps = msg.steps
        time_ = msg.time
        interrupt = msg.interrupt
        relative_height = msg.relative_height
        try:
            if gait > 0:
                self.controller.set_step_mode(
                    gait,
                    stride,
                    height,
                    direction,
                    rotation,
                    time_,
                    steps,
                    interrupt=interrupt,
                    relative_height=relative_height)
            else:
                if gait == 0:
                    self.controller.stop_running(timeout=None, callback=lambda:self.controller.set_pose(None, None, time_))
                elif gait == -1:
                    self.controller.stop_running()
                elif gait == -2:
                    self.controller.set_build_in_pose('DEFAULT_POSE', time_)
                else:
                    pass
        except Exception as e:
            rospy.logerr(str(e))

    def set_leg_absolute_cb(self, leg_pos: jetmsg.LegPosition):
        leg_id = leg_pos.leg_id
        leg_pos = leg_pos.position.x, leg_pos.position.y, leg_pos.position.z
        duration = leg_pos.duration
        self.controller.set_leg_position(leg_id, leg_pos, duration)


    def set_leg_relatively_cb(self, leg_pos: jetmsg.LegPosition):
        leg_id = leg_pos.leg_id
        duration = leg_pos.duration
        leg_pos = leg_pos.position.x, leg_pos.position.y, leg_pos.position.z
        cur_pos = list(self.controller.pose[leg_id - 1])
        new_pos = cur_pos[0] + leg_pos[0], cur_pos[1] + leg_pos[1], cur_pos[2] + leg_pos[2]
        self.controller.set_leg_position(leg_id, new_pos, duration)


    def odometry_publish(self, event):
        cur_time = rospy.Time.now()
        cur_quat = R.from_euler('xyz', [-self.controller.transform[1][0], -self.controller.transform[1][1], self.controller.pose_yaw], False).as_quat()
        cur_position = self.controller.position

        # odom transform message begin
        # transformation from odom to base_link
        odom_trans = TransformStamped()
        odom_trans.header.stamp = cur_time
        odom_trans.header.frame_id = "".join([self.tf_prefix, "odom"])
        odom_trans.child_frame_id =  "".join([self.tf_prefix, "base_link"])

        # translation and rotation
        translation, rotation = Vector3(), Quaternion()
        translation.x, translation.y, translation.z = cur_position
        rotation.x, rotation.y, rotation.z, rotation.w = cur_quat
        odom_trans.transform.translation, odom_trans.transform.rotation = translation, rotation
        self.odom_trans_pub.publish(odom_trans)
        # odom transform message end
        #
        odom = nav_msgs.Odometry()
        odom.header.stamp = cur_time
        odom.header.frame_id = "".join([self.tf_prefix, "odom"])
        odom.child_frame_id =  "".join([self.tf_prefix, "base_link"])
        # set the positions
        position, orientation = Point(), Quaternion()
        position.x, position.y, position.z = cur_position
        orientation.x, orientation.y, orientation.z, orientation.w = cur_quat
        odom.pose.pose.position, odom.pose.pose.orientation = position, orientation
        #
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[14] = 1000000.0
        odom.pose.covariance[21] = 1000000.0
        odom.pose.covariance[28] = 1000000.0
        odom.pose.covariance[35] = 1000.0

        # set the velocity
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = self.controller.linear_x, self.controller.linear_y
        odom.twist.twist.angular.z = self.controller.angular_z
        odom.twist.covariance = odom.pose.covariance
        #
        twist = TwistWithCovarianceStamped()
        twist.header.frame_id = "odom"
        twist.header.stamp = cur_time
        twist.twist.twist.linear.x, twist.twist.twist.linear.y = (self.controller.linear_x, self.controller.linear_y)
        twist.twist.twist.angular.z = self.controller.angular_z
        twist.twist.covariance = odom.pose.covariance
        self.odometry_pub.publish(odom)
        self.twist_pub.publish(twist)
        self.last_time_odometry = cur_time

    #call the built-in posture
    def set_pose1_cb(self, req: SetPose1Request):
        rospy.loginfo("set_pose_1 called")
        rsp = SetPose1Response()
        try:
            self.controller.set_build_in_pose(req.pose, req.duration)
        except Exception as e:
            rospy.logerr(str(e))
            rsp.result = -1
            rsp.msg = str(e)
        finally:
            return rsp

    def set_pose2_cb(self, req: SetPose2Request):
        rospy.loginfo("set_pose_2 called")
        rsp = SetPose2Response()
        try:
            pose = [(point.x, point.y, point.z) for point in req.pose]
            self.controller.set_pose(pose, req.duration, interrupt=req.interrupt)
        except Exception as e:
            rospy.logerr(str(e))
            rsp.result = -1
            rsp.msg = str(e)
        return rsp
    
    def set_pose_euler_cb(self, msg: jetmsg.Pose):
        self.controller.transform_absolutely(
            (msg.position.x, msg.position.y, msg.position.z), 
            (msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw), 0.4)

    def pose_transform_1_cb(self, msg: PoseTransformRequest):
        translation = msg.translation
        rotation = msg.rotation
        duration = msg.duration
        try:
            self.controller.transform_pose(translation, rotation, duration)
        except Exception as e:
            rospy.logerr(str(e))

    def pose_transform_euler_cb(self, msg: jetmsg.TransformEuler):
        translation = msg.translation.x, msg.translation.y, msg.translation.z
        rotation = msg.rotation.x, msg.rotation.y, msg.rotation.z
        duration = msg.duration

        try:
            self.controller.transform_pose_2(translation, "xyz", rotation, duration, degrees=False)
        except Exception as e:
            rospy.logerr(str(e))

    def run_action_set_sub_cb(self, msg: jetmsg.RunActionSet):
        rospy.loginfo("{}, {}".format(msg.action_path, msg.repeat))
        file_path = '/home/hiwonder/ActionSets/' + msg.action_path if msg.default_path else msg.action_path
        self.controller.run_action_set(file_path, msg.repeat)


def main():
    jethexa_controller_node = jethexaControlNode('jethexa_control')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == '__main__':
    main()
