#! /usr/bin/env python
# -*- coding: utf-8 -*-
import threading
from datetime import datetime
import agf_mc_protocol

plc = agf_mc_protocol
import socket
from time import sleep
from logging import debug
from bson.json_util import dumps
import os
import sys
import rospy
import rospkg
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy
import actionlib
import time
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
import requests
import yaml
from geometry_msgs.msg import (
    Twist,
    Pose,
    PoseStamped,
    Quaternion,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
import json
from std_msgs.msg import Bool, Int16, Int8, String, Float32
from math import sqrt, pow, pi, sin, cos, atan2, degrees
from actionlib_msgs.msg import GoalStatus
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse
from cognex_qr_code.srv import *

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)
from mongodb import mongodb, LogLevel, MissionStatus
from enum import Enum
from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    lockup_pose,
    offset_pose_x,
    offset_pose_yaw,
    SENSOR_DEACTIVATE,
    SENSOR_ACTIVATE,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    offset_pose_xy_theta,
    distance_two_pose,
    YamlDumper,
    distance_two_points,
)

from os.path import expanduser
from agv_msgs.msg import ErrorRobotToPath

HOME = expanduser("~")

def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )
    parser.add_option(
        "-c",
        "--config_path",
        dest="config_path",
        default=os.path.join(rospkg.RosPack().get_path("matehan"), "cfg"),
    )
    parser.add_option(
        "-r",
        "--robot_config_file",
        dest="robot_config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("amr_config"),
            "cfg",
            "control_system",
            "robot_config.yaml",
        ),
    )
    parser.add_option(
        "--robot_define",
        dest="robot_define",
        default=os.path.join(
            HOME,
            "robot_config",
            "robot_define.yaml",
        ),
    )
    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def connect_plc(plc_address, port):
    """Check if PLC is reachable via TCP"""
    try:
        with socket.create_connection((plc_address, port), timeout=3.0) as sock:
            local_ip, local_port = sock.getsockname()
            rospy.loginfo("PLC is reachable at {}:{}".format(plc_address, port))
            rospy.loginfo("Connected from local {}:{}".format(local_ip, local_port))
            sleep(5)
            sock.close()
            return True
    except socket.timeout:
        rospy.logwarn("PLC connection to {}:{} timed out after 3s.".format(plc_address, port))
    except socket.error as e:
        rospy.logerr("PLC socket error at {}:{} — {}".format(plc_address, port, e))
    except Exception as e:
        rospy.logerr("Unexpected error when checking PLC connection: {}".format(e))

    return False  # Moved outside the try block for clarity

def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("elevator_test_2", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    # ElevatorAction(rospy.get_name(), **vars(options))
    while not rospy.is_shutdown():
        connect_plc("10.23.13.37", 2004)
        sleep(15)



if __name__ == "__main__":
    main()
