import rospy
from jethexa_controller_interfaces.msg import Traveling, TransformEuler, LegPosition, TransformEuler, RunActionSet, Pose
from jethexa_controller_interfaces.srv import SetPose1, SetPose1Request
from geometry_msgs.msg import Twist


class Client:
    def __init__(self, node):
        self.node = node
        self.traveling_pub = rospy.Publisher("jethexa_controller/traveling", Traveling, queue_size=1)
        self.head_absolute_pub = rospy.Publisher("jethexa_controller/set_head_absolute", TransformEuler, queue_size=1)
        self.head_relatively_pub = rospy.Publisher("jethexa_controller/set_head_relatively", TransformEuler, queue_size=1)
        self.leg_absolute_pub = rospy.Publisher("jethexa_controller/set_leg_absolute", LegPosition, queue_size=1)
        self.leg_relatively_pub = rospy.Publisher("jethexa_controller/set_leg_relatively", LegPosition, queue_size=1)
        self.transform_euler_pub = rospy.Publisher("jethexa_controller/pose_transform_euler", TransformEuler, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("jethexa_controller/cmd_vel", Twist, queue_size=1)
        self.run_actionset_pub = rospy.Publisher("jethexa_controller/run_actionset", RunActionSet, queue_size=1)
        self.set_build_in_pose_proxy = rospy.ServiceProxy("jethexa_controller/set_pose_1", SetPose1)
        self.set_pose_euler_sub = rospy.Publisher("jethexa_controller/set_pose_euler", Pose, queue_size=1)

    def pose_transform_euler(self, translation, euler, duration):
        msg = TransformEuler()
        msg.translation.x, msg.translation.y, msg.translation.z = translation
        msg.rotation.x, msg.rotation.y, msg.rotation.z = euler
        msg.duration = duration
        self.transform_euler_pub.publish(msg)

    def set_head_absolute(self, pitch: float, yaw: float, duration):
        msg = TransformEuler()
        msg.rotation.y = float(pitch)
        msg.rotation.z = float(yaw)
        msg.duration = duration
        self.head_absolute_pub.publish(msg)

    def set_head_relatively(self, pitch: float, yaw: float, duration):
        msg = TransformEuler()
        msg.rotation.y = float(pitch)
        msg.rotation.z = float(yaw)
        msg.duration = duration
        self.head_relatively_pub.publish(msg)

    def set_leg_absolute(self, leg_id, x, y, z, duration):
        msg = LegPosition()
        msg.leg_id = leg_id
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.duration = duration
        self.leg_absolute_pub.publish(msg)

    def set_leg_relatively(self, leg_id, x, y, z, duration):
        msg = LegPosition()
        msg.leg_id = leg_id
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.duration = duration
        self.leg_relatively_pub.publish(msg)

    def traveling(self,
                  gait=1,
                  stride=30.0,
                  height=15.0,
                  direction=0.0,
                  rotation=0.0,
                  time=0.6,
                  steps=1,
                  interrupt=True,
                  relative_height=False):
        msg = Traveling()
        msg.gait = gait
        msg.stride = float(stride)
        msg.height = float(height)
        msg.direction = float(direction)
        msg.rotation = float(rotation)
        msg.time = float(time)
        msg.steps = steps
        msg.interrupt = interrupt
        msg.relative_height = relative_height
        self.traveling_pub.publish(msg)

    def cmd_vel(self, v_x, v_y, a_z):
        msg = Twist()
        msg.linear.x = v_x
        msg.linear.y = v_y
        msg.angular.z = a_z
        self.cmd_vel_pub.publish(msg)

    def set_build_in_pose(self, pose_name, duration):
        return self.set_build_in_pose_proxy(pose=pose_name, duration=duration)

    def run_actionset(self, action_name, repeat, default_path=True):
        msg = RunActionSet()
        msg.action_path = action_name
        msg.repeat = repeat
        msg.default_path = default_path
        self.run_actionset_pub.publish(msg)

    def set_pose_euler(self, trans, rotate):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = trans
        msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw = rotate
        self.set_pose_euler_sub.publish(msg)


