import rospy
import math
import sqlite3
from jethexa_sdk import serial_servo
from jethexa_controller import build_in_pose, kinematics_api


def load_actionset(file_path):
    with sqlite3.connect(file_path) as con:
        acts = con.execute("SELECT * FROM ActionGroup")
        new_acts = [act[1:] for act in acts]
        print(new_acts)
        return new_acts


def do_action(action):
    duration = action[0]
    for servo_id, pos in enumerate(action[1:]):
        serial_servo.set_position(servo_id + 1, pos, duration)
    return duration

SPEC = {
}


def square(self):
        """
        表演模式的动作， 虚拟动作组名称 "___square"
        """
        rospy.loginfo(":DSFSDF")
        self.set_pose_base(build_in_pose.DEFAULT_POSE, 0.5)
        steps = 15
        step_time = 0.6
        for i in range(5):
            rospy.sleep(0.1)
            if self.stopping:
                return
        self.set_step_mode_base(1, 40, 12, 0, 0, step_time, steps)
        if self.stopping:
            return
        self.set_step_mode_base(1, 40, 12, math.radians(-90), 0, step_time, steps)
        if self.stopping:
            return
        self.set_step_mode_base(1, 40, 12, math.radians(180), 0, step_time, steps)
        if self.stopping:
            return
        self.set_step_mode_base(1, 40, 12, math.radians(90), 0, step_time, steps)
        if self.stopping:
            return




def wave(self):
        """
        表演模式的动作， 虚拟动作组名称 "___wave"
        """
        duration = 0.03
        self.set_pose_base(build_in_pose.DEFAULT_POSE_M, 0.8)
        org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
        rospy.sleep(0.8)
        # Gradually speed up and increase the swing
        for j in range(7, 20, 2):
            i = 90 
            j = min(15, j)
            while i <= 360 + 85 and not self.stopping: 
                if i == 90 and j == 7:
                    t = 0.5
                else:
                    t = duration
                i += 4 + j * 0.30
                x = math.sin(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                y = math.cos(math.radians(i)) * (0.018 * (j + ((i - 90) / 360) * 2))
                pose = kinematics_api.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.set_pose_base(pose, t)
                rospy.sleep(t)

        # Gradually slow down and reduce the swing
        for j in range(15, 4, -3):
            i = 360 + 85
            while i >= 90 and not self.stopping:
                i += -(4 + j * 0.30)
                k = 360 + 90 - i + 90
                x = math.sin(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                y = math.cos(math.radians(k)) * (0.018 * (j + (1 - (i - 90) / 360) * -3))
                pose = kinematics_api.transform_euler(org_pose, (0, 0, 0), 'xy', (x, y), degrees=False)
                self.set_pose_base(pose, duration)
                rospy.sleep(duration)
        # return to normal
        self.set_pose_base(build_in_pose.DEFAULT_POSE_M, 1)
    
def turn_round(self):
        """
        表演模式的虚拟动作组， 扭身
        """
        self.set_pose_base(build_in_pose.DEFAULT_POSE_M, 0.8)
        org_pose = tuple(build_in_pose.DEFAULT_POSE_M)
        rospy.sleep(0.8)
        pose = kinematics_api.transform_euler(org_pose, (0, 0, 30), 'xyz', (0, 0, 0.8), degrees=False)
        print(pose)
        self.set_pose_base(pose, 0.3)
        for i in range(0, 7):
            pose = kinematics_api.transform_euler(org_pose, (0, 0, -30), 'xyz', (0, 0, -0.8), degrees=False)
            self.set_pose_base(pose, 0.6)
            rospy.sleep(0.6)
            pose = kinematics_api.transform_euler(org_pose, (0, 0, 30), 'xyz', (0, 0, 0.8), degrees=False)
            self.set_pose_base(pose, 0.6)
            rospy.sleep(0.6)
        self.set_pose_base(build_in_pose.DEFAULT_POSE_M, 1)

def actionset_runner(controller, file_path, repeat):
    """
    运行动作组
    有一些特殊的名称不对应实际的动作组文件， 而是用代码实现的，请注意
    :param file_path: 文件名/文件路径
    :param repeat: 动作组重复的次数, 0为无限循环
    """
    if file_path in SPEC:
        return

    forever = False if repeat != 0 else True # determine whether it loops infinitely
    acts = load_actionset(file_path) # read the action group files 
    print(acts)

    # execute the actions based on the action group data
    while repeat > 0 or forever:
        for act in acts:
            yield act
        repeat = repeat - 1 if repeat > 0 else 0

