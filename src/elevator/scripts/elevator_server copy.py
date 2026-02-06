#! /usr/bin/env python
# -*- coding: utf-8 -*-
import threading
from datetime import datetime
import agf_mc_protocol

plc = agf_mc_protocol

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


class MainState(EnumString):
    NONE = -1
    SEND_DOCKING_HUB = 0
    DOCKING_TO_HUB = 1
    CHECK_CART = 2
    LIFT_MAX = 3
    LIFT_MIN = 4
    DONE = 8
    MOVING_ERROR = 10
    PAUSED = 12
    WAITING = 13
    SEND_GOTO_WAITING = 14
    GOING_TO_WAITING = 15
    SEND_GOTO_OUT_OF_HUB = 23
    GOING_TO_OUT_OF_HUB = 24
    MOVING_DISCONNECTED = 28
    INIT = 29
    LIFT_POSITION_WRONG = 30
    NO_CART = 31
    OPTICAL_SENSOR_ERROR = 32
    EMG_AGV = 33
    LECH_TAM = 34
    ALIGNMENT_SENSOR = 35
    LIFT_MIN_END = 36
    LIFT_MIN_FIRST = 37
    LIFT_MAX_FIRST = 38
    UPDATE_CART_ERROR = 44
    UNABLE_PLACE_CART = 47
    WRONG_CART = 48
    PAUSED_BY_ELEVATOR = 49
    COLLISION_POSSIBLE = 50
    EMG_ELEVATOR = 60
    TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE = 70
    NETWORK_ERROR = 71
    WAIT_RESET_IO = 72


class RunType(Enum):
    NONE = -1
    GO_NOMAL = 0
    GO_DOCKING = 1
    GO_OUT_DOCKING = 2
    STOP_ACCURACY = 3
    STOP_BY_CROSS_LINE = 4


class MainStatePlace(EnumString):
    NONE = -1
    CHECK_ELEVATOR_POSIBLE = 200
    REQUEST_ENTER_ELEVATOR = 201
    CHECK_ENTER_POSSIBLE = 202
    CHECK_AGV_PLACE_COMPLETE = 203
    DONE_CARRY_IN = 204
    CARRY_IN_POSSIBLE = 205


class MainStatePick(EnumString):
    NONE = -1
    CHECK_ELEVATOR_POSIBLE = 100
    REQUEST_ENTER_ELEVATOR = 101
    CHECK_ENTER_POSSIBLE = 102
    CHECK_AGV_PICK_COMPLETE = 103
    DONE_CARRY_OUT = 104
    PAUSE = 105


class ElevatorName(EnumString):
    ELEVATOR_A = 0
    ELEVATOR_B = 1
    ELEVATOR_C = 2
    ELEVATOR_D = 3


PICK = 1
PLACE = 0
ON = 1
OFF = 0
LIFT_UP = 1
LIFT_DOWN = 2
FAKE_QR_CODE = False
FORWARD = 1
BACKWARD = 0

# INPUT PLC
carry_in_request = [1030, 1031, 1032]
carry_in_completed = [1040, 1041, 1042]
carry_out_request = [1050, 1051, 1052]
carry_out_completion = [1060, 1061, 1062]
destination_ST_indication = [100, 102, 104]
carry_in_ID = [120, 121, 122]
request_door_open = [1090, 1091, 1092]
id_floor_destination = [1001, 1002, 1003]

# OUTPUT PLC
carry_in_instruction_possible = [1020, 1021, 1022]
carry_in_possible = [1050, 1051, 1052]
loading_and_unloading = [1060, 1061, 1062]
carrying_out_possible = [1070, 1071, 1072]
completion_ACK = [1080, 1081, 1082]
carry_out_ID = [30, 31, 32]
autorator_enter_stop = 1005
emg_elevator = 1001
# fmt: off
x_value_address = [
    1000,
    1001,
    1002,
    1003,
    1004,
    1030,
    1031,
    1032,
    1040,
    1041,
    1042,
    1050,
    1051,
    1052,
    1060,
    1061,
    1062,
    1070
]
y_value_address = [
    1004,
    1005,
    1006,
    1020,
    1021,
    1022,
    1050,
    1051,
    1052,
    1060,
    1061,
    1062,
    1070,
    1071,
    1072,
    1080,
    1081,
    1082,
]
# fmt: on
w_value_address_input = [100, 101, 102, 103, 104, 105, 120, 121, 122]
w_value_address_output = [15, 30, 31, 32]

pre_x_value = []
pre_y_value = []
pre_w_value_input = []
pre_w_value_output = []

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


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("elevator_test", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    # ElevatorAction(rospy.get_name(), **vars(options))
    plc.plc_connect("10.23.12.39", 1004)


if __name__ == "__main__":
    main()
