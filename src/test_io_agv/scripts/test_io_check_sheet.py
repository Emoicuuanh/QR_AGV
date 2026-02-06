#! /usr/bin/env python
# -*- coding: utf-8 -*-
from pyclbr import Function
import yaml
import os
import sys
import rospy
import rospkg
import numpy as np
from math import *
from nav_msgs.msg import Odometry
from agv_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import threading
import copy
import json
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import (
    Twist,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from std_msgs.msg import Bool, Int16, String, Int16MultiArray, Int8
from math import pi, sqrt
from tf.listener import TransformListener, Transformer
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)
from common_function import make_transform_stamped, angle_two_pose
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
    Float32Stamped,
    Int16MultiArrayStamped,
)
from cognex_qr_code.srv import *
from agv_msgs.msg import ArduinoIO, DigitalSensor, FollowLineSensor, LedControl
from agv_msgs.msg import (
    FollowLineSensor,
    DiffDriverMotorSpeed,
    EncoderDifferential,
)
from common_function import (
    EnumString,
    lockup_pose,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_info,
    print_warn,
    print_error,
    obj_to_dict,
    angle_robot_vs_robot_to_goal,
    distance_two_pose,
    YamlDumper,
)

SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1

LIFT_UP = 0
LIFT_DOWN = 1


class MainState(EnumString):
    NONE = -1
    DONE = 0
    CHECK_FOLLOWLINE = 1
    CHECK_CARD_ID = 2
    CHECK_MOTOR = 3
    CHECK_QR_CODE = 4
    CHECK_FRONT_START_FIRST = 5
    CHECK_FRONT_START_SECOND = 6
    CHECK_FRONT_STOP_FIRST = 7
    CHECK_FRONT_STOP_SECOND = 8
    CHECK_EMG_FISRT = 9
    CHECK_EMG_RELEASE = 10
    CHECK_BACK_START_FIRST = 11
    CHECK_BACK_START_SECOND = 13
    CHECK_BACK_STOP_FIRST = 14
    CHECK_BACK_STOP_SECOND = 15
    CHECK_BUMPER_FIRST = 16
    CHECK_BUMPER_SECOND = 17
    CHECK_LIFT_UP = 18
    CHECK_LIFT_DOWN = 19
    CHECK_DETECT_VRACK_SENSOR = 20


class Button(EnumString):
    BUTTON_FRONT_START = 1
    BUTTON_FRONT_STOP = 2
    BUTTON_BACK_START = 3
    BUTTON_BACK_STOP = 4
    EMG = 5
    BUMPER = 6


class LedTurn(EnumString):
    FRONT_RIGHT = 1
    FRONT_LEFT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4
    FRONT = 5
    BACK = 6
    ALL = 7
    OFF_FRONT_RIGHT = 8
    OFF_BACK_LEFT = 9
    OFF_BACK_RIGHT = 10
    OFF_FRONT = 11
    OFF_BACK = 12
    OFF_ALL = 13
    RIGHT = 14
    LEFT = 15
    OFF_RIGHT = 16
    OFF_LEFT = 17


class TEST_IO_AGV(object):
    def __init__(self, name, *args, **kwargs):
        # """
        # ..######..##.....##.########...######..########..####.########..########.########.
        # .##....##.##.....##.##.....##.##....##.##.....##..##..##.....##.##.......##.....##
        # .##.......##.....##.##.....##.##.......##.....##..##..##.....##.##.......##.....##
        # ..######..##.....##.########..##.......########...##..########..######...########.
        # .......##.##.....##.##.....##.##.......##...##....##..##.....##.##.......##...##..
        # .##....##.##.....##.##.....##.##....##.##....##...##..##.....##.##.......##....##.
        # ..######...#######..########...######..##.....##.####.########..########.##.....##
        # """

        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/card_id", String, self.card_id_cb)
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber(
            "/followline_sensor", FollowLineSensor, self.followline_cb
        )
        rospy.Subscriber(
            "/motor_encoder", EncoderDifferential, self.motor_encoder_cb
        )

        # """
        # .########..##.....##.########..##.......####..######..##.....##.########.########.
        # .##.....##.##.....##.##.....##.##........##..##....##.##.....##.##.......##.....##
        # .##.....##.##.....##.##.....##.##........##..##.......##.....##.##.......##.....##
        # .########..##.....##.########..##........##...######..#########.######...########.
        # .##........##.....##.##.....##.##........##........##.##.....##.##.......##...##..
        # .##........##.....##.##.....##.##........##..##....##.##.....##.##.......##....##.
        # .##.........#######..########..########.####..######..##.....##.########.##.....##
        # """

        self.pub_lift_cmd = rospy.Publisher(
            "/lift_cart", Int8Stamped, queue_size=10
        )
        self.pub_led_turn = rospy.Publisher(
            "/arduino_driver/led_turn", Int16MultiArrayStamped, queue_size=10
        )
        self.fastech_control_pub = rospy.Publisher(
            "/fastech_control_multiarray", Int16MultiArray, queue_size=10
        )
        self.pub_led_status = rospy.Publisher(
            "/led_status", StringStamped, queue_size=5, latch=True
        )
        rospy.Subscriber(
            "/fastech_input", Int16MultiArray, self.optical_sensor_cb
        )
        rospy.Subscriber(
            "/fastech_output", Int16MultiArray, self.read_fastech_output_cb
        )
        # """
        # ..######..########.########..##.....##.####..######..########
        # .##....##.##.......##.....##.##.....##..##..##....##.##......
        # .##.......##.......##.....##.##.....##..##..##.......##......
        # ..######..######...########..##.....##..##..##.......######..
        # .......##.##.......##...##....##...##...##..##.......##......
        # .##....##.##.......##....##....##.##....##..##....##.##......
        # ..######..########.##.....##....###....####..######..########
        # """
        self.get_qr_code = rospy.ServiceProxy("ReadQrCode", QrCode)

        self.init_variable(*args, **kwargs)
        self.loop()

    def yaml_load(self, filepath):
        try:
            with open(filepath, "r") as read_file:
                data = yaml.load(read_file, Loader=yaml.FullLoader)
            return data
        except Exception as e:
            rospy.logerr("Load yaml file error: {}".format(e))

    def init_variable(self, *args, **kwargs):
        self.rate = rospy.Rate(10)
        config_file_led = kwargs["config_file_led"]
        rospy.loginfo("config_file_led: %s" % config_file_led)
        led_effect_config = self.yaml_load(config_file_led)
        self.led_effect_list = led_effect_config["led_effect"]
        self.led_turn = [0, 0, 0, 0]
        self.liftup_finish = False
        self.liftdown_finish = False
        self.emg_status = False
        self.start_1 = False
        self.start_2 = False
        self.stop_1 = False
        self.stop_2 = False
        self.detect_vrack = False
        self.bumper = False
        self.pre_liftup_finish = False
        self.pre_liftdown_finish = False
        self.pre_emg_status = False
        self.pre_start_1 = False
        self.pre_start_2 = False
        self.pre_stop_1 = False
        self.pre_stop_2 = False
        self.pre_detect_vrack = False
        self.pre_bumper = False
        self.led_number = 0
        self.led_turn_msg = Int16MultiArrayStamped()
        self.led_msg = StringStamped()
        self.optical_msg = Int16MultiArray()
        self.lift_msg = Int8Stamped()
        self.followline = ""
        self.card_id = ""
        self.motor = ""
        self.pose_odom2robot = Pose()
        self.qr_code = ""
        self.control_lift_up = False
        self.control_lift_down = False
        self.thread_optical = threading.Thread(
            name="thread_publish", target=self.optical_sensor
        )
        self.thread_optical.daemon = (
            True  # Stop thread when main thread stopped.
        )
        self.thread_optical.start()

    def shutdown(self):
        rospy.loginfo("Shuting down")

    """
    .########.##.....##.##....##..######..########.####..#######..##....##
    .##.......##.....##.###...##.##....##....##.....##..##.....##.###...##
    .##.......##.....##.####..##.##..........##.....##..##.....##.####..##
    .######...##.....##.##.##.##.##..........##.....##..##.....##.##.##.##
    .##.......##.....##.##..####.##..........##.....##..##.....##.##..####
    .##.......##.....##.##...###.##....##....##.....##..##.....##.##...###
    .##........#######..##....##..######.....##....####..#######..##....##
    """

    def loop(self):
        _state = MainState.CHECK_FOLLOWLINE
        _prev_state = MainState.NONE
        self.led_turn_control(LedTurn.OFF_ALL)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            if _prev_state != _state:
                rospy.loginfo(
                    "MODULE CHECK STATE: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
            if _state == MainState.CHECK_FOLLOWLINE:
                if self.followline != "":
                    rospy.logwarn("FOLLOWLINE OK")
                    pre_odom_x = self.pose_odom2robot.position.x
                    _state = MainState.CHECK_MOTOR
                    self.led_turn_control(LedTurn.FRONT)
            elif _state == MainState.CHECK_MOTOR:
                if abs(self.pose_odom2robot.position.x - pre_odom_x) > 0.1:
                    rospy.logwarn("MOTOR OK")
                    _state = MainState.CHECK_CARD_ID
                    self.led_turn_control(LedTurn.RIGHT)
            elif _state == MainState.CHECK_CARD_ID:
                if self.card_id != "":
                    rospy.logwarn("RFID OK")
                    _state = MainState.CHECK_QR_CODE
                    self.led_turn_control(LedTurn.BACK)
            elif _state == MainState.CHECK_QR_CODE:
                 # resp = self.get_qr_code(2)
                # self.qr_code = resp.Res
                # if self.qr_code != "":
                rospy.sleep(3)
                if True:
                    rospy.logwarn("QR CODE OK")
                    _state = MainState.CHECK_FRONT_START_FIRST
                    self.led_turn_control(LedTurn.LEFT)
            elif _state == MainState.CHECK_FRONT_START_FIRST:
                if self.check_click_button(Button.BUTTON_FRONT_START):
                    rospy.logwarn("FRONT START OK")
                    self.led_turn_control(LedTurn.FRONT_RIGHT)
                    _state = MainState.CHECK_FRONT_START_SECOND
            elif _state == MainState.CHECK_FRONT_START_SECOND:
                if self.check_click_button(Button.BUTTON_FRONT_START):
                    rospy.logwarn("FRONT START OK")
                    self.led_turn_control(LedTurn.OFF_ALL)
                    _state = MainState.CHECK_FRONT_STOP_FIRST
            elif _state == MainState.CHECK_FRONT_STOP_FIRST:
                if self.check_click_button(Button.BUTTON_FRONT_STOP):
                    rospy.logwarn("FRONT STOP OK")
                    self.led_turn_control(LedTurn.FRONT_LEFT)
                    _state = MainState.CHECK_FRONT_STOP_SECOND
            elif _state == MainState.CHECK_FRONT_STOP_SECOND:
                if self.check_click_button(Button.BUTTON_FRONT_STOP):
                    rospy.logwarn("FRONT STOP OK")
                    self.led_turn_control(LedTurn.OFF_ALL)
                    _state = MainState.CHECK_BACK_STOP_FIRST
            elif _state == MainState.CHECK_BACK_STOP_FIRST:
                if self.check_click_button(Button.BUTTON_BACK_STOP):
                    rospy.logwarn("BACK STOP OK")
                    self.led_turn_control(LedTurn.BACK_LEFT)
                    _state = MainState.CHECK_BACK_STOP_SECOND
            elif _state == MainState.CHECK_BACK_STOP_SECOND:
                if self.check_click_button(Button.BUTTON_BACK_STOP):
                    rospy.logwarn("BACK STOP OK")
                    self.led_turn_control(LedTurn.OFF_ALL)
                    _state = MainState.CHECK_BACK_START_FIRST
            elif _state == MainState.CHECK_BACK_START_FIRST:
                if self.check_click_button(Button.BUTTON_BACK_START):
                    rospy.logwarn("BACK START OK")
                    self.led_turn_control(LedTurn.BACK_RIGHT)
                    _state = MainState.CHECK_BACK_START_SECOND
            elif _state == MainState.CHECK_BACK_START_SECOND:
                if self.check_click_button(Button.BUTTON_BACK_START):
                    rospy.logwarn("BACK START OK")
                    self.led_turn_control(LedTurn.OFF_ALL)
                    _state = MainState.CHECK_EMG_FISRT
            elif _state == MainState.CHECK_EMG_FISRT:
                if not self.emg_status:
                    rospy.logwarn("EMG ON OK")
                    self.led_turn_control(LedTurn.FRONT)
                    _state = MainState.CHECK_EMG_RELEASE
            elif _state == MainState.CHECK_EMG_RELEASE:
                if self.emg_status:
                    rospy.logwarn("EMG OFF OK")
                    self.led_turn_control(LedTurn.OFF_ALL)
                    _state = MainState.CHECK_BUMPER_FIRST
            elif _state == MainState.CHECK_BUMPER_FIRST:
                if self.check_click_button(Button.BUMPER):
                    rospy.logwarn("BUMPER OK")
                    self.led_turn_control(LedTurn.BACK)
                    _state = MainState.CHECK_BUMPER_SECOND
            elif _state == MainState.CHECK_BUMPER_SECOND:
                if self.check_click_button(Button.BUMPER):
                    rospy.logwarn("BUMPER OK")
                    self.led_turn_control(LedTurn.OFF_ALL)
                    _state = MainState.CHECK_LIFT_UP
            elif _state == MainState.CHECK_LIFT_UP:
                if self.check_click_button(Button.BUTTON_FRONT_START):
                    self.control_lift_up = True
                if self.control_lift_up:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_UP
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self.liftup_finish:
                    rospy.logwarn("LIFT UP OK")
                    self.led_test("MANUAL")
                    _state = MainState.CHECK_DETECT_VRACK_SENSOR
            elif _state == MainState.CHECK_DETECT_VRACK_SENSOR:
                # if self.detect_vrack:
                if True:
                    rospy.sleep(3)
                    rospy.logwarn("DETECT VRACK OK")
                    self.led_test("OFF")
                    _state = MainState.CHECK_LIFT_DOWN
            elif _state == MainState.CHECK_LIFT_DOWN:
                if not self.detect_vrack:
                    self.control_lift_down = True
                if self.control_lift_down:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self.liftdown_finish:
                    _state = MainState.DONE
            elif _state == MainState.DONE:
                self.led_test("MANUAL")
                self.led_turn_control(LedTurn.ALL)
                sys.exit()

            self.pre_start_1 = self.start_1
            self.pre_start_2 = self.start_2
            self.pre_stop_1 = self.stop_1
            self.pre_stop_2 = self.stop_2
            self.pre_emg_status = self.emg_status
            self.pre_bumper = self.bumper
            self.rate.sleep()

    def led_turn_control(self, led_type):
        if led_type == LedTurn.FRONT_RIGHT:
            self.led_turn_msg.data = [0, 1, 0, 0]
        elif led_type == LedTurn.FRONT_LEFT:
            self.led_turn_msg.data = [1, 0, 0, 0]
        elif led_type == LedTurn.BACK_RIGHT:
            self.led_turn_msg.data = [0, 0, 0, 1]
        elif led_type == LedTurn.BACK_LEFT:
            self.led_turn_msg.data = [0, 0, 1, 0]
        elif led_type == LedTurn.FRONT:
            self.led_turn_msg.data = [1, 1, 0, 0]
        elif led_type == LedTurn.BACK:
            self.led_turn_msg.data = [0, 0, 1, 1]
        elif led_type == LedTurn.RIGHT:
            self.led_turn_msg.data = [0, 1, 1, 0]
        elif led_type == LedTurn.LEFT:
            self.led_turn_msg.data = [1, 0, 0, 1]
        elif led_type == LedTurn.ALL:
            self.led_turn_msg.data = [1, 1, 1, 1]
        else:
            self.led_turn_msg.data = [0, 0, 0, 0]
        self.led_turn_msg.stamp = rospy.Time.now()
        self.pub_led_turn.publish(self.led_turn_msg)
        rospy.sleep(0.1)

    def check_click_button(self, type_button):
        if type_button == Button.BUTTON_FRONT_START:
            if (
                self.start_1 == SENSOR_ACTIVATE
                and self.pre_start_1 == SENSOR_DEACTIVATE
            ):
                return True
            else:
                return False
        elif type_button == Button.BUTTON_FRONT_STOP:
            if (
                self.stop_1 == SENSOR_ACTIVATE
                and self.pre_stop_1 == SENSOR_DEACTIVATE
            ):
                return True
            else:
                return False
        elif type_button == Button.BUTTON_BACK_STOP:
            if (
                self.stop_2 == SENSOR_ACTIVATE
                and self.pre_stop_2 == SENSOR_DEACTIVATE
            ):
                return True
            else:
                return False
        elif type_button == Button.BUTTON_BACK_START:
            if (
                self.start_2 == SENSOR_ACTIVATE
                and self.pre_start_2 == SENSOR_DEACTIVATE
            ):
                return True
            else:
                return False
        elif type_button == Button.EMG:
            if (
                self.emg_status == SENSOR_DEACTIVATE
                and self.pre_emg_status == SENSOR_ACTIVATE
            ):
                return True
            else:
                return False
        elif type_button == Button.BUMPER:
            if (
                self.bumper == SENSOR_DEACTIVATE
                and self.pre_bumper == SENSOR_ACTIVATE
            ):
                return True
            else:
                return False

    def optical_sensor(self):
        while True:
            for i in range(8):
                list = [0, 0, 0, 0, 0, 0, 0, 0]
                list[i] = 1
                self.optical_msg.data = list
                self.fastech_control_pub.publish(self.optical_msg)
                rospy.sleep(1)

    def led_test(self, led_name):
        self.led_msg.stamp = rospy.Time.now()
        try:
            self.led_msg.data = led_name
            self.pub_led_status.publish(self.led_msg)
            rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr("Control led error error: {}".format(e))

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def odom_cb(self, msg):
        self.pose_odom2robot = msg.pose.pose

    def read_fastech_output_cb(self, msg):
        self.read_fastech_value_fb = list(msg.data)

    def optical_sensor_cb(self, msg):
        self.digial_input = msg

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "lift_max_sensor" in data: self.liftup_finish = data["lift_max_sensor"]
        if "lift_min_sensor" in data: self.liftdown_finish = data["lift_min_sensor"]
        if "emg_button" in data: self.emg_status = data["emg_button"]
        if "start_1_button" in data: self.start_1 = data["start_1_button"]
        if "start_2_button" in data: self.start_2 = data["start_2_button"]
        if "stop_1_button" in data: self.stop_1 = data["stop_1_button"]
        if "stop_2_button" in data: self.stop_2 = data["stop_2_button"]
        if "detect_vrack" in data: self.detect_vrack = data["detect_vrack"]
        if "bumper" in data: self.bumper = data["bumper"]

    def motor_encoder_cb(self, msg):
        self.motor = msg.left

    def followline_cb(self, msg):
        FollowLineCounter = 0
        for i in range(16):
            FollowLineCounter += msg.data[i]
        if FollowLineCounter != 0:
            self.followline = msg.data

    def card_id_cb(self, msg):
        self.card_id = msg.data


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
        "--config_file_led",
        dest="config_file_led",
        default=os.path.join(
            rospkg.RosPack().get_path("amr_config"), "cfg", "led_effect.yaml"
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
    rospy.init_node("test_io_agv", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    TEST_IO_AGV(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
