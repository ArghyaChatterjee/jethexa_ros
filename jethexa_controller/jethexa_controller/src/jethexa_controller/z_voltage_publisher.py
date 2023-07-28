import rospy
import jethexa_sdk.serial_servo as serial_servo
import std_msgs.msg


# publish voltage
class VoltagePublisher:

    def __init__(self, node, rate=1):
        self.__node = node
        self.publisher = rospy.Publisher('voltage', std_msgs.msg.Float32, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.callback)

    def callback(self, timer_event):
        # rospy.loginfo("received new update")
        if self.__node.controller.voltage != 0:
            msg = std_msgs.msg.Float32()
            msg.data = self.__node.controller.voltage
            self.publisher.publish(msg)
