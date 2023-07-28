import rospy
from sensor_msgs.msg import JointState


# publish joint angle
class JointStatesPublisher:

    def __init__(self, node, rate=10):
        self.__node = node
        self.publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.callback)

    def callback(self, timer_event):
        #rospy.loginfo("update_joint_states")
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        joints_state = self.__node.controller.joints_state
        msg.name = list(k for k in joints_state.keys())

        msg.position = list(joints_state.values())
        self.publisher.publish(msg)
