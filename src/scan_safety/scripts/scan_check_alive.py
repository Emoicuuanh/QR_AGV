#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import rospkg
import os
import yaml
try:
    rospack = rospkg.RosPack()
    common_func_dir = os.path.join(rospack.get_path("agv_common_library"), "scripts")
    if not os.path.isdir(common_func_dir):
        common_func_dir = os.path.join(rospack.get_path("agv_common_library"), "release")
    sys.path.insert(0, common_func_dir)
except Exception as e:
    print(f"Error finding agv_common_library: {e}")
from module_manager import ModuleClient
from std_msgs.msg import Bool

class ModuleMonitor(object):
    def __init__(self):
        self.pub_vel_alive_module = rospy.Publisher(
            "/disable_motor_disconnect",Bool,queue_size=10
        )
        self.module_list = []
        self.module_config = [] # Khởi tạo rỗng trước
        self.robot_config = {}

        # Gọi load config ngay trong init hoặc phải đảm bảo gọi trước khi dùng module_config
        self.load_config()
        self.init_modules()

    def load_config(self):
        try:
            default_path = os.path.join(
                rospkg.RosPack().get_path("amr_config"), "cfg", "control_system", "robot_config.yaml"
            )
        except Exception:
            default_path = "" # Xử lý trường hợp không tìm thấy gói

        self.config_path = rospy.get_param("~robot_config_file", default_path)
        rospy.loginfo(f"Loading robot config from: {self.config_path}")

        if not os.path.exists(self.config_path):
            rospy.logerr(f"Không tìm thấy file config: {self.config_path}")
            return False

        with open(self.config_path) as f:
            config_dict = yaml.load(f, Loader=yaml.FullLoader)
            if config_dict is None:
                rospy.logerr(f"File config trống: {self.config_path}")
            else:
                self.robot_config = config_dict
                self.module_config = self.robot_config.get("module_list", [])

    def init_modules(self):
        # Tách phần khởi tạo module ra hàm riêng để rõ ràng
        for i in self.module_config:
            # i là dictionary, ví dụ: {'lidar_node': {...config...}}
            if not isinstance(i, dict): continue
            try:
                client_module = ModuleClient(
                    list(i.keys())[0], list(i.values())[0]
                )
                self.module_list.append(client_module)
            except Exception as e:
                rospy.logerr(f"Failed to init module")

    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.count_alive = 0
            for module in self.module_list:
                if not module.module_alive:
                    rospy.logerr(f"Module {module.display_name} is not alive!")
                    self.count_alive = self.count_alive + 1
            if self.count_alive != 0:
                self.stop_msg = True
                self.pub_vel_alive_module.publish(self.stop_msg)
            else:
                self.stop_msg = False
                self.pub_vel_alive_module.publish(self.stop_msg)
            rate.sleep()


def main():
    rospy.init_node("module_check_alive", anonymous=False)
    monitor = ModuleMonitor()
    # Không cần gọi monitor.load_config() ở đây nữa vì đã gọi trong __init__
    monitor.loop()

if __name__ == "__main__":
    main()