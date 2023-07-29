import rospy
import threading
import std_srvs.srv
import lab_config.srv


class LabConfigProxy:
    def __init__(self):
        self.heartbeat_timer = None
        self.node = rospy.init_node('lab_config_client', log_level=rospy.INFO)

    def heartbeat(self):
        proxy = rospy.ServiceProxy('/lab_config_manager/heartbeat', std_srvs.srv.SetBool)
        proxy(True)
        self.heartbeat_timer = threading.Timer(2, self.heartbeat)
        self.heartbeat_timer.start()

    def enter_func(self):
        proxy = rospy.ServiceProxy('/lab_config_manager/enter', std_srvs.srv.Trigger)
        proxy()
        self.heartbeat()

    def exit_func(self):
        proxy = rospy.ServiceProxy('/lab_config_manager/exit', std_srvs.srv.Trigger)
        proxy()
        self.heartbeat_timer.cancel()

    def get_all_color_name(self):
        proxy = rospy.ServiceProxy('/lab_config_manager/get_all_color_name', lab_config.srv.GetAllColorName)
        ret = proxy()
        return ret.color_names

    def get_range_by_name(self, name):
        proxy = rospy.ServiceProxy('/lab_config_manager/get_range', lab_config.srv.GetRange)
        ret: lab_config.srv.GetRangeResponse = proxy(name)
        return dict(min=list(ret.min), max=list(ret.max))

    def set_current_range(self, range_min, range_max):
        proxy = rospy.ServiceProxy('/lab_config_manager/change_range', lab_config.srv.ChangeRange)
        ret = proxy(min=range_min, max=range_max)
        return ret

    def apply_current_range(self, name):
        proxy = rospy.ServiceProxy('/lab_config_manager/stash_range', lab_config.srv.StashRange)
        ret = proxy(color_name=name)
        return ret

    def save_ranges_to_disk(self):
        proxy = rospy.ServiceProxy('/lab_config_manager/save_to_disk', std_srvs.srv.Trigger)
        proxy()
