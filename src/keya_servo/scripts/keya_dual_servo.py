#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import json
import sys
import rospy
import rospkg
import yaml
from std_msgs.msg import Int32, Int16, Int8, String, Empty
from geometry_msgs.msg import Twist
import serial
from math import pi
from datetime import datetime
from nav_msgs.msg import Odometry
from agv_msgs.msg import EncoderDifferential
from safety_msgs.msg import SafetyStatus
from std_stamped_msgs.msg import StringStamped, Float32Stamped
from std_msgs.msg import Bool
common_folder = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_folder):
    common_folder = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_folder)
from common_function import find_soft_port
from module_manager import ModuleClient
LEFT = 1
RIGHT = 2

msg_odom = Odometry()


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class KeyaDiffDriver:
    def __init__(self):
        rospy.init_node("keya_dual_servo")
        rospy.loginfo("Init node keya_dual_servo")

        self.init_varialble()
        self.init_ros()
        self.init_serial()
        self.load_config()
        self.init_modules()
        self.set_controller_params()
        self.poll()


    def init_varialble(self):
        self.port = rospy.get_param("~port", "3-1")
        self.baud = rospy.get_param("~baud", 115200)
        rospy.loginfo("Set port: {}".format(self.port))
        self.port = find_soft_port(self.port)
        rospy.loginfo("Connecting port: {}".format(self.port))

        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.0)
        self.wheel_separation = rospy.get_param("~wheel_separation", 0.0)

        self.cmd_vel_timeout = rospy.get_param("~cmd_vel_timeout", 0.3)

        self.lin_vel_ratio = rospy.get_param("~lin_vel_ratio", 1.0)
        self.ang_vel_ratio = rospy.get_param("~ang_vel_ratio", 1.0)
        self.vel_max = rospy.get_param("~vel_max", 1.2)
        self.use_encoder = rospy.get_param("~use_encoder", True)
        self.motor_direction = rospy.get_param("~motor_direction", False)
        self.motor_position_wrong = rospy.get_param("~motor_position_wrong", False)
        self.motor_mirror = rospy.get_param("~motor_mirror", True)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 100000)
        self.max_ampe = rospy.get_param("~max_ampe", 100000)
        self.time_check_motor_not_run = rospy.get_param("~time_check_motor_not_run", 1)

        self.keep_alive_topic = rospy.get_param(
            "~keep_alive/topic", "moving_control/module_status"
        )
        self.keep_alive_timeout = rospy.get_param("~keep_alive/timeout", -1.0)

        self.left_vel = 0.0
        self.right_vel = 0.0
        self.fm_fault_active = False
        self.fm_fault_start_time = 0.0
        self.fm_fault_paused = False  # Track if mission was paused due to FM fault
        self.fm_fault_clear_time = 0.0  # Track when fault was cleared



        self.last_encoder_received = rospy.get_time()
        self.last_topic_alive = rospy.get_time()
        self.last_cmd_vel_time = rospy.get_time()
        self.last_send_cmd_vel_time = rospy.get_time()
        self.last_time = 0
        self.motor_time = rospy.get_time()
        self.last_safety_time = rospy.get_time()
        # Encoder
        self.int_data_encoder_right = 0
        self.int_data_encoder_left = 0
        self.pre_stick_right = 0
        self.now_stick_right = 0
        self.pulse_encoder_right_offset = 0
        self.subtract_right = 0
        self.encoder_right_value = int()

        self.pre_stick_left = 0
        self.pulse_encoder_left_offset = 0
        self.encoder_left_value = int()
        self.now_stick_left = 0
        self.subtract_left = 0
        self.flag_right = 0
        self.flag_left = 0

        #module alive
        self.module_list = []
        self.module_config = [] # Khởi tạo rỗng trước
        self.robot_config = {}
        self.error_start_time = None
        self.module_error_stop = False


        # Reset encoder
        self.pre_time_left = 0
        self.pre_time_right = 0
        self.i_test_odom_straight = 0

        self.encoder_left_diff = 0.0
        self.pre_encoder_left = 0.0
        self.encoder_right_diff = 0.0
        self.pre_encoder_right = 0.0
        # Motor
        self.motor_revert = False
        self.step = 0

        self.left_vel_pre = 0.0
        self.right_vel_pre = 0.0
        self.frequency_motor = 20.0
        self.time_motor_pre = 0.0

        self.autocontroll = False
        self.manualcontroll = False
        self.is_safety = False
        # Robot status
        self.robot_status = None
        self.status_robot = ""
        self.mode_robot = ""

        self.last_time_control_right_success = rospy.Time.now()
        self.last_time_control_left_success = rospy.Time.now()
        self.vel_left_target = 0
        self.vel_right_target = 0
        self.disable_control_motor_left = False
        self.disable_control_motor_right = False
        self.pre_error_value_left = 0
        self.pre_error_value_right = 0
        self.rad_per_tick = (2 * pi) / float(self.encoder_resolution)

        self.encoder_msg = EncoderDifferential()
        self.ampe_right_msg = Float32Stamped()
        self.ampe_left_msg = Float32Stamped()

        self.record_first_encoder_left = True
        self.record_first_encoder_right = True

        self.emg_status = True
        self.bumper = True

        self.disable_left = False
        self.disable_right = False

        self.delay_check_motor_left_after_emg_bumper = False
        self.delay_check_motor_right_after_emg_bumper = False
        self.time_delay_motor_left = rospy.get_time()
        self.time_delay_motor_right = rospy.get_time()
        self.time_disable_read_ampe = rospy.get_time()
        self.max_ampe_receive = 0

    def set_controller_params(self):
        # TODO: Read params again to confirm
        # Channel 1
        # Accelerate
        cmd = b"^MAC" + b" " + b"1" + b" " + b"32000" + b"\r"
        self.serial_port.write(cmd)
        # Decelerate
        cmd = b"^MDEC" + b" " + b"1" + b" " + b"32000" + b"\r"
        self.serial_port.write(cmd)
        # Channel 2
        cmd = b"^MAC" + b" " + b"2" + b" " + b"32000" + b"\r"
        self.serial_port.write(cmd)
        cmd = b"^MDEC" + b" " + b"2" + b" " + b"32000" + b"\r"
        self.serial_port.write(cmd)

#########################################
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
#########################################################

    def init_serial(self):
        # Init serial port
        max_attempts = 10  # Maximum number of retry attempts
        attempt = 0

        while attempt < max_attempts:
            try:
                self.serial_port = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    parity="N",
                    stopbits=1,
                    bytesize=8,
                    timeout=0.1,
                )
                if self.serial_port.isOpen():
                    rospy.loginfo("Connected to port: {}".format(self.serial_port.portstr))
                    return  # Exit the function if connection is successful
            except Exception as e:
                rospy.logerr("Serial connect fail: {}".format(e))
                rospy.logerr("Attempt {} of {} failed. Retrying in 1 seconds...".format(attempt + 1, max_attempts))
                rospy.sleep(1)
                attempt += 1

        rospy.logerr("Failed to connect to serial port after {} attempts.".format(max_attempts))
        # Optional: handle failure to connect, e.g., by shutting down the node or raising an exception
        # rospy.signal_shutdown("Unable to connect to serial port.")

    def init_ros(self):
        rospy.Subscriber(
            "/fake_disable_motor", StringStamped, self.fake_disable_motor_cb
        )
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/reset_encoder", Empty, self.reset_encoder_cb)
        rospy.Subscriber(
            self.keep_alive_topic, StringStamped, self.keep_alive_cb
        )
        rospy.Subscriber("/safety_status", SafetyStatus, self.safety_status_cb)
        rospy.Subscriber("/robot_status", StringStamped, self.robot_status_cb)
        self.pub_mission_run_pause = rospy.Publisher(
            "/mission_manager/run_pause_req", StringStamped, queue_size=10
        )
        self.safety_vel_pub = rospy.Publisher(
            "/safety_cmd_vel", Twist, queue_size=10
        )
        self.encoder_pub = rospy.Publisher(
            "/motor_encoder", EncoderDifferential, queue_size=10
        )
        self.motor_status_pub = rospy.Publisher(
            "motor_status", String, queue_size=10
        )
        self.error_control_left_pub = rospy.Publisher(
            "/error_control_motor_left", Int16, queue_size=10
        )
        self.error_control_right_pub = rospy.Publisher(
            "/error_control_motor_right", Int16, queue_size=10
        )
        self.error_status_pub = rospy.Publisher(
            "/error_motor_status", StringStamped, queue_size=10
        )
        self.ampe_left = rospy.Publisher(
            "/ampe_left", Float32Stamped, queue_size=10
        )
        self.ampe_right = rospy.Publisher(
            "/ampe_right", Float32Stamped, queue_size=10
        )
        rospy.on_shutdown(self.shutdown)

    """

      ####    ##   #      #      #####    ##    ####  #    #
     #    #  #  #  #      #      #    #  #  #  #    # #   #
     #      #    # #      #      #####  #    # #      ####
     #      ###### #      #      #    # ###### #      #  #
     #    # #    # #      #      #    # #    # #    # #   #
      ####  #    # ###### ###### #####  #    #  ####  #    #

    """

    def fake_disable_motor_cb(self, msg):
        data = msg.data
        if data == "":
            self.disable_left = False
            self.disable_right = False
            rospy.logwarn("NOT DISABLE")
        elif data == "left":
            self.disable_left = True
            rospy.logwarn("DISABLE MOTOR LEFT")
        elif data == "right":
            self.disable_right = True
            rospy.logwarn("DISABLE MOTOR RIGHT")
        elif data == "all":
            self.disable_left = True
            rospy.logwarn("DISABLE ALL")
            self.disable_right = True

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "bumper" in data: self.bumper = data["bumper"]
        if "emg_button" in data: self.emg_status = data["emg_button"]

    def cmd_vel_cb(self, msg):
        if self.module_error_stop:
            return
        self.last_cmd_vel_time = rospy.get_time()
        self.lin_vel = msg.linear.x * self.lin_vel_ratio
        self.ang_vel = msg.angular.z * self.ang_vel_ratio

        if self.motor_direction:
            self.left_vel = self.lin_vel - (
                self.ang_vel * self.wheel_separation / 2
            )
            self.right_vel = self.lin_vel + (
                self.ang_vel * self.wheel_separation / 2
            )

        else:
            self.left_vel = -(
                self.lin_vel - (self.ang_vel * self.wheel_separation / 2)
            )
            self.right_vel = -(
                self.lin_vel + (self.ang_vel * self.wheel_separation / 2)
            )
        if self.left_vel >= self.vel_max:
            self.left_vel = self.vel_max
        elif self.left_vel <= -self.vel_max:
            self.left_vel = -self.vel_max
        if self.right_vel >= self.vel_max:
            self.right_vel = self.vel_max
        elif self.right_vel <= -self.vel_max:
            self.right_vel = -self.vel_max

    def reset_encoder_cb(self, msgs):
        self.reset_encoder()

    def keep_alive_cb(self, msg):
        self.last_topic_alive = rospy.get_time()
    def safety_status_cb(self, msg):
        self.last_safety_time = rospy.get_time()
        safety_fields = list(msg.fields)  # [1, 2, 3, ...]
        if len(safety_fields) > 0:
            if safety_fields[0] == 1:
                self.is_safety = True
            else:
                self.is_safety = False
        else:
            self.is_safety = False

    def robot_status_cb(self, msg):
        self.robot_status = json.loads(msg.data)
        if "status" in self.robot_status:
            self.status_robot = self.robot_status["status"]
        if "mode" in self.robot_status:
            self.mode_robot = self.robot_status["mode"]


    """

     ###### #    # #    #  ####  ##### #  ####  #    #
     #      #    # ##   # #    #   #   # #    # ##   #
     #####  #    # # #  # #        #   # #    # # #  #
     #      #    # #  # # #        #   # #    # #  # #
     #      #    # #   ## #    #   #   # #    # #   ##
     #       ####  #    #  ####    #   #  ####  #    #

    """

    def check_and_reconnect_serial(self):
        if not self.serial_port.isOpen():
            rospy.logwarn("Wait for reconect serial")
            rospy.sleep(0.5)
            self.init_serial()
            rospy.sleep(0.5)
            return False
        return True

    def tick_to_distance(self, tick):
        return tick * self.rad_per_tick * (self.wheel_diameter / 2)

    def check_motor_left_not_run(self):
        if not self.emg_status or not self.bumper:
            self.delay_check_motor_left_after_emg_bumper = True
            self.time_delay_motor_left = rospy.get_time()
            self.record_first_encoder_left = True
        else:
            if abs(self.left_vel) > 0.01:
                if self.delay_check_motor_left_after_emg_bumper:
                    if rospy.get_time() - self.time_delay_motor_left > 1:
                        self.delay_check_motor_left_after_emg_bumper = False
                    else:
                        return
                if (
                    self.disable_control_motor_left
                    or self.disable_control_motor_right
                ):
                    return
                if self.record_first_encoder_left:
                    self.encoder_left_begin = self.encoder_msg.left
                    self.time_left_begin = rospy.get_time()
                    self.record_first_encoder_left = False
                else:
                    if (rospy.get_time() - self.time_left_begin) > self.time_check_motor_not_run:
                        self.record_first_encoder_left = True
                        # if self.tick_to_distance(abs(self.encoder_msg.left - self.encoder_left_begin)) < 0.01:
                        if abs(self.left_vel) > 0.09:
                            if (
                                abs(
                                    self.encoder_msg.left
                                    - self.encoder_left_begin
                                )
                                < 50
                            ):
                                # self.disable_control_motor_left = True
                                self.error_control_left_pub.publish(1)
                                rospy.logerr_throttle(3,"Motor left control fail")
                            else:
                                self.disable_control_motor_left = False
                                self.error_control_left_pub.publish(0)
                        else:
                            if (
                                abs(
                                    self.encoder_msg.left
                                    - self.encoder_left_begin
                                )
                                < 10
                            ):
                                # self.disable_control_motor_left = True
                                self.error_control_left_pub.publish(1)
                                rospy.logerr_throttle(3,"Motor left control fail")
                            else:
                                self.disable_control_motor_left = False
                                self.error_control_left_pub.publish(0)
            else:
                self.disable_control_motor_left = False
                self.record_first_encoder_left = True
                self.error_control_left_pub.publish(0)

    def check_motor_right_not_run(self):
        if not self.emg_status or not self.bumper:
            self.delay_check_motor_right_after_emg_bumper = True
            self.time_delay_motor_right = rospy.get_time()
            self.record_first_encoder_right = True

        else:
            if abs(self.right_vel) > 0.01:
                if self.delay_check_motor_right_after_emg_bumper:
                    if rospy.get_time() - self.time_delay_motor_right > 1:
                        self.delay_check_motor_right_after_emg_bumper = False
                    else:
                        return
                if (
                    self.disable_control_motor_left
                    or self.disable_control_motor_right
                ):
                    return
                if self.record_first_encoder_right:
                    self.encoder_right_begin = self.encoder_msg.right
                    self.time_right_begin = rospy.get_time()
                    self.record_first_encoder_right = False
                else:
                    if (rospy.get_time() - self.time_right_begin) > self.time_check_motor_not_run:
                        self.record_first_encoder_right = True
                        # if self.tick_to_distance(abs(self.encoder_msg.right - self.encoder_right_begin)) < 0.01:
                        if abs(self.right_vel) > 0.09:
                            if (
                                abs(
                                    self.encoder_msg.right
                                    - self.encoder_right_begin
                                )
                                < 50
                            ):
                                # self.disable_control_motor_right = True
                                self.error_control_right_pub.publish(1)
                                rospy.logerr_throttle(3,"Motor right control fail")
                            else:
                                self.disable_control_motor_right = False
                                self.error_control_right_pub.publish(0)
                        else:
                            if (
                                abs(
                                    self.encoder_msg.right
                                    - self.encoder_right_begin
                                )
                                < 10
                            ):
                                # self.disable_control_motor_right = True
                                self.error_control_right_pub.publish(1)
                                rospy.logerr_throttle(3,"Motor right control fail")
                            else:
                                self.disable_control_motor_right = False
                                self.error_control_right_pub.publish(0)

            else:
                self.record_first_encoder_right = True
                self.disable_control_motor_right = False
                self.error_control_right_pub.publish(0)

    def shutdown(self):
        self.serial_port.write(b"!M" + b" " + b"0" + b" " + b"0" + b"\r")
        self.serial_port.close()
        print("ros shutdown in {}".format(rospy.get_name()))

    def control_motor(self, vel_left, vel_right):
        self.last_send_cmd_vel_time = rospy.get_time()
        if self.motor_mirror:
            self.vel_left_target = -1.0 * (
            vel_left * 1000
        ) / 1.57  # 1.57 = v_control * 1000 / v_max

        else:
            self.vel_left_target = (
                vel_left * 1000
            ) / 1.57  # 1.57 = v_control * 1000 / v_max
        self.vel_right_target = (vel_right * 1000) / 1.57
        a = str(self.vel_left_target).encode("utf-8")
        b = str(self.vel_right_target).encode("utf-8")
        if self.motor_position_wrong:
            cmd = b"!M" + b" " + a + b" " + b + b"\r"
        else:
            cmd = b"!M" + b" " + b + b" " + a + b"\r"
        self.serial_port.write(cmd)

    def read_encoder(self):
        now = datetime.now()
        t1 = rospy.get_time()
        self.serial_port.write(b"?C\r")
        self.serial_port.flush()  # Day data ra
        # self.data_encoder = self.serial_port.inWaiting()  # doc so byte trong input buffer
        self.last_encoder_received = rospy.get_time()
        while True:
            # self.data_encoder_receive = self.serial_port.readline()
            self.data_encoder_receive = self.serial_port.read_until(b"\r")
            self.data_encoder_receive = self.data_encoder_receive.decode(
                "UTF-8"
            )
            # rospy.logwarn(self.data_encoder_receive)
            if "C=" in self.data_encoder_receive:
                # try:

                x = self.data_encoder_receive.find("=")
                y = self.data_encoder_receive.find(":")
                if self.motor_position_wrong:
                    self.encoder_left_value = int(
                        self.data_encoder_receive[x + 1 : y]
                    )
                    self.encoder_right_value = int(
                        self.data_encoder_receive[y + 1 :]
                    )
                else:
                    self.encoder_right_value = int(
                        self.data_encoder_receive[x + 1 : y]
                    )
                    self.encoder_left_value = int(
                        self.data_encoder_receive[y + 1 :]
                    )

                if self.motor_mirror:
                    self.encoder_msg.right = int(
                        -1 * (self.encoder_right_value - self.pulse_encoder_right_offset)
                        / 4
                    )
                else:
                    self.encoder_msg.right = int(
                        (self.encoder_right_value - self.pulse_encoder_right_offset)
                        / 4
                    )
                self.encoder_msg.left = int(
                    (self.encoder_left_value - self.pulse_encoder_left_offset)
                    / 4
                )

                self.encoder_msg.header.stamp = rospy.Time.now()
                self.encoder_pub.publish(self.encoder_msg)
                # except:
                # rospy.logwarn("Read encoder data false")
                break
            if (rospy.get_time() - self.last_encoder_received) > 0.5:
                break
        # rospy.logwarn_throttle(5, rospy.get_time() - t1)

    def reset_encoder(self):
        self.pulse_encoder_left_offset = self.encoder_left_value
        self.pulse_encoder_right_offset = self.encoder_right_value
        rospy.loginfo("Reset encoder")

    def read_volate(self):
        self.serial_port.write(b"?V\r")
        self.serial_port.flush()  # Day data ra
        last_voltage_received = rospy.get_time()
        while True:
            data_voltage_receive = self.serial_port.read_until(b"\r")
            data_voltage_receive = data_voltage_receive.decode("UTF-8")
            if "V=" in data_voltage_receive:
                rospy.logwarn(data_voltage_receive)
                break
            if (rospy.get_time() - last_voltage_received) > 0.5:
                break

    def read_amps(self):
        self.serial_port.write(b"?A\r")
        self.serial_port.flush()  # Day data ra
        last_amps_received = rospy.get_time()
        while True:
            data_amps_receive = self.serial_port.read_until(b"\r")
            data_amps_receive = data_amps_receive.decode("UTF-8")
            if "A=" in data_amps_receive:
                x = data_amps_receive.find("=")
                y = data_amps_receive.find(":")
                self.ampe_left_msg.data = abs(int(data_amps_receive[x + 1 : y]) / 10)
                self.ampe_left_msg.stamp = rospy.Time.now()
                self.ampe_right_msg.data = abs(int(data_amps_receive[y + 1 :]) / 10)
                self.ampe_right_msg.stamp = rospy.Time.now()
                if self.ampe_left_msg.data > self.max_ampe:
                    self.disable_control_motor_left = True
                    self.time_disable_read_ampe = rospy.get_time()
                    self.max_ampe_receive = self.ampe_left_msg.data
                    rospy.logerr_throttle(3, "Motor left over current")
                if self.ampe_right_msg.data > self.max_ampe:
                    self.disable_control_motor_right = True
                    self.time_disable_read_ampe = rospy.get_time()
                    self.max_ampe_receive = self.ampe_right_msg.data
                    rospy.logerr_throttle(3, "Motor right over current")
                if (rospy.get_time() - self.time_disable_read_ampe) < 0.5:
                    self.ampe_left_msg.data = self.max_ampe_receive
                    self.ampe_right_msg.data = self.max_ampe_receive
                self.ampe_left.publish(self.ampe_left_msg)
                self.ampe_right.publish(self.ampe_right_msg)
                break
            if (rospy.get_time() - last_amps_received) > 0.5:
                break

    def read_battery_amps(self):
        self.serial_port.write(b"?BA\r")
        self.serial_port.flush()  # Day data ra
        last_battery_amps_received = rospy.get_time()
        while True:
            data_battery_amps_receive = self.serial_port.read_until(b"\r")
            data_battery_amps_receive = data_battery_amps_receive.decode(
                "UTF-8"
            )
            if "BA=" in data_battery_amps_receive:
                break
            if (rospy.get_time() - last_battery_amps_received) > 0.5:
                break

    def read_FM(self):
        self.serial_port.write(b"?FM\r")
        self.serial_port.flush()

        last_FM_received = rospy.get_time()

        while True:
            data = self.serial_port.read_until(b"\r")
            try:
                data = data.decode("UTF-8").strip()
            except:
                continue

            if data.startswith("FM="):
                try:
                    fm_data = data.split("=")[1]

                    # ----- Dual motor -----
                    if ":" in fm_data:
                        fm_left_str, fm_right_str = fm_data.split(":")
                        fm_left = int(fm_left_str)
                        fm_right = int(fm_right_str)
                        # LEFT motor
                        if fm_left != 0:
                            faults_left = self.decode_FM_flag(fm_left)
                            rospy.logerr(
                                "[FM LEFT] value=%d | %s"
                                % (fm_left, ", ".join(faults_left))
                            )
                        else:
                            rospy.loginfo("[FM LEFT] No fault")

                        # RIGHT motor
                        if fm_right != 0:
                            faults_right = self.decode_FM_flag(fm_right)
                            rospy.logerr(
                                "[FM RIGHT] value=%d | %s"
                                % (fm_right, ", ".join(faults_right))
                            )
                        else:
                            rospy.loginfo("[FM RIGHT] No fault")
            
                        if fm_left != 0 or fm_right != 0:
                            if not self.fm_fault_active:
                                # Fault just became active
                                self.fm_fault_active = True
                                self.fm_fault_start_time = rospy.get_time()
                                rospy.logerr("[FM FAULT] Detected -> Pausing mission")
                                # Send pause command if in AUTO mode
                                if self.status_robot == "RUNNING" and self.mode_robot == "AUTO":
                                    self.pub_mission_run_pause.publish(
                                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                                    )
                                    self.fm_fault_paused = True
                        else:
                            if self.fm_fault_active:
                                # Fault just cleared
                                self.fm_fault_active = False
                                self.fm_fault_clear_time = rospy.get_time()
                                rospy.logwarn("[FM CLEAR] Fault cleared -> Will resume after 3 seconds")

                    # ----- Single motor (fallback) -----
                    else:
                        fm_value = int(fm_data)
                        if fm_value != 0:
                            faults = self.decode_FM_flag(fm_value)
                            rospy.logerr(
                                "[FM] value=%d | %s"
                                % (fm_value, ", ".join(faults))
                            )
                        else:
                            rospy.loginfo("[FM] No fault")
                            
                        if fm_value != 0:
                            if not self.fm_fault_active:
                                # Fault just became active
                                self.fm_fault_active = True
                                self.fm_fault_start_time = rospy.get_time()
                                rospy.logerr("[FM FAULT] Detected -> Pausing mission")
                                # Send pause command if in AUTO mode
                                if self.status_robot == "RUNNING" and self.mode_robot == "AUTO":
                                    self.pub_mission_run_pause.publish(
                                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                                    )
                                    self.fm_fault_paused = True
                        else:
                            if self.fm_fault_active:
                                # Fault just cleared
                                self.fm_fault_active = False
                                self.fm_fault_clear_time = rospy.get_time()
                                rospy.logwarn("[FM CLEAR] Fault cleared -> Will resume after 3 seconds")

                except Exception as e:
                    rospy.logerr("FM parse error: %s | raw=%s" % (str(e), data))

                break

            if rospy.get_time() - last_FM_received > 0.1:
                rospy.logwarn("Timeout read_FM")
                break

    def read_status(self):
        self.serial_port.write(b"?FS\r")
        self.serial_port.flush()
        last_status_received = rospy.get_time()
        while True:
            data_status_receive = self.serial_port.read_until(b"\r")
            data_status_receive = data_status_receive.decode("UTF-8")
            if "FS=" in data_status_receive:
                x = data_status_receive.find("=")
                status_value = int(data_status_receive[x + 1 :])
                if status_value != 0:
                    status = self.decode_status_flag(status_value)
                    status_str = ", ".join(status) if status else "Unknown status"
                    if status:
                        rospy.logerr("[StatuFlag] Status value: %d, Details: %s" % (status_value, status_str))
                    else:
                        rospy.loginfo("[StatuFlag] Status value: %d, No active faults." % status_value)
                else:
                #     # self.error_status_pub.publish("No error")
                    rospy.loginfo("[StatuFlag] No status.")
                break
            elapsed = rospy.get_time() - last_status_received
            if elapsed > 0.1:
                rospy.logwarn("Timeout read_status: elapsed %.3f seconds" % elapsed)
                break

    def read_error(self):
        self.serial_port.write(b"?FF\r")
        self.serial_port.flush()
        last_error_received = rospy.get_time()
        while True:
            data_error_receive = self.serial_port.read_until(b"\r")
            data_error_receive = data_error_receive.decode("UTF-8")
            if "FF=" in data_error_receive:
                x = data_error_receive.find("=")
                error_value = int(data_error_receive[x + 1 :])
                if error_value != 0:
                    faults = self.decode_fault_flag(error_value)
                    fault_str = ", ".join(faults) if faults else "Unknown error"
                    msg = StringStamped(stamp=rospy.Time.now(), data=fault_str)
                    self.error_status_pub.publish(msg)
                    if faults:
                        rospy.logerr("[FaultFlag] Error value: %d, Details: %s" % (error_value, fault_str))
                    else:
                        rospy.loginfo("[FaultFlag] Error value: %d, No active faults." % error_value)
                else:
                    # self.error_status_pub.publish("No error")
                    rospy.logwarn("[FaultFlag] No error.")
                break
            elapsed = rospy.get_time() - last_error_received
            if elapsed > 0.1:
                rospy.logwarn("Timeout read_error: elapsed %.3f seconds" % elapsed)
                break

    def read_encoder_speed(self):
        self.serial_port.write(b"?S\r")
        self.serial_port.flush()  # Day data ra
        last_encoder_speed_received = rospy.get_time()
        while True:
            encoder_speed_receive = self.serial_port.read_until(b"\r")
            encoder_speed_receive = encoder_speed_receive.decode("UTF-8")
            if "S=" in encoder_speed_receive:
                x = encoder_speed_receive.find("=")
                y = encoder_speed_receive.find(":")
                encoder_left_speed = int(encoder_speed_receive[x + 1 : y])
                encoder_right_speed = int(encoder_speed_receive[y + 1 :])
                rospy.logwarn(
                    "encoder_left_speed :{}".format(encoder_left_speed)
                )
                rospy.logerr(
                    "encoder_right_speed :{}".format(encoder_right_speed)
                )
            if (rospy.get_time() - last_encoder_speed_received) > 0.5:
                break

    def read_close_loop_error(self):
        self.serial_port.write(b"?E\r")
        self.serial_port.flush()  # Day data ra
        last_error_close_loop_received = rospy.get_time()
        while True:
            data_close_loop_error_receive = self.serial_port.read_until(b"\r")
            data_close_loop_error_receive = (
                data_close_loop_error_receive.decode("UTF-8")
            )
            if "E=" in data_close_loop_error_receive:
                x = data_close_loop_error_receive.find("=")
                y = data_close_loop_error_receive.find(":")
                error_value_left = int(data_close_loop_error_receive[x + 1 : y])
                error_value_right = int(data_close_loop_error_receive[y + 1 :])
                rospy.logwarn_throttle(
                    1, "error_value_left :{}".format(error_value_left)
                )
                rospy.logerr_throttle(
                    1, "vel_left_target :{}".format(self.vel_left_target)
                )
                if (
                    self.pre_error_value_left == error_value_left
                    and self.vel_left_target != 0
                    and error_value_left != 0
                ):
                    if (
                        rospy.Time.now() - self.last_time_control_left_success
                    ).to_sec() >= 0.5:
                        self.error_control_left_pub.publish(0)
                        self.serial_port.write(
                            b"!M" + b" " + b"0" + b" " + b"0" + b"\r"
                        )
                        self.disable_control_motor_left = True
                else:
                    self.pre_error_value_left = error_value_left
                    self.last_time_control_left_success = rospy.Time.now()
                    self.error_control_left_pub.publish(1)
                    self.disable_control_motor_left = False
                if (
                    self.pre_error_value_right == error_value_right
                    and self.vel_right_target != 0
                    and error_value_right != 0
                ):
                    if (
                        rospy.Time.now() - self.last_time_control_right_success
                    ).to_sec() >= 0.5:
                        self.error_control_right_pub.publish(0)
                        self.serial_port.write(
                            b"!M" + b" " + b"0" + b" " + b"0" + b"\r"
                        )
                        self.disable_control_motor_right = True
                else:
                    self.pre_error_value_right = error_value_right
                    self.last_time_control_right_success = rospy.Time.now()
                    self.error_control_right_pub.publish(1)
                    self.disable_control_motor_right = False
                break
            if (rospy.get_time() - last_error_close_loop_received) > 0.5:
                break

    def decode_fault_flag(self, error_value):
        faults = [
            "Overheat",                # f1
            "Overvoltage",             # f2
            "Undervoltage",            # f3
            "Short circuit",           # f4
            "Emergency stop",          # f5
            "Sepex excitation fault",  # f6
            "MOSFET failure",          # f7
            "Startup config fault"     # f8
        ]
        active_faults = []
        for i, fault in enumerate(faults):
            if error_value & (1 << i):
                active_faults.append(fault)
        return active_faults

    def decode_FM_flag(self, error_value):
        faults = [
            "Amps Limit current active",                # f1
            "Motor stalled",             # f2
            "Loop Error detected",            # f3
            "Safety Stop active",           # f4
            "Forward Limit triggered",          # f5
            "Revesre Limit triggered",  # f6
            "Amps trigger activated"         # f7
        ]
        active_faults = []
        for i, fault in enumerate(faults):
            if error_value & (1 << i):
                active_faults.append(fault)
        return active_faults

    def decode_status_flag(self, error_value):
        faults = [
            "Serial mode"                 #f1
            "Pulse mode",                # f2
            "Analog mode",             # f3
            "Power stage off",            # f4
            "Stall detected",           # f5
            "At limit",          # f6
            "Unused",  # f7
            "Micro Basic script running"          # f8
        ]
        active_faults = []
        for i, fault in enumerate(faults):
            if error_value & (1 << i):
                active_faults.append(fault)
        return active_faults


    def poll(self):
        PULLING_FREQ = 30
        NOP_TIME = 0.0001
        rate = rospy.Rate(40)
        t = rospy.get_time()
        diff_time = 1.0 / PULLING_FREQ
        self.time_motor_pre = rospy.get_time()
        safety_stop = False
        prev_safety_stop = False
        last_safety_stop_time = rospy.get_time()
        begin_safety_stop = rospy.get_time()
        safety_vel = Twist()
        safety_vel.linear.x = 0.0
        safety_vel.angular.z = 0.0
        last_time_motor_disconected = rospy.get_time()
        reset_error = False
        last_error_check_time = rospy.get_time()

        while not rospy.is_shutdown():
            try:
                # Check and reconnect serial port if disconnected
                if not self.check_and_reconnect_serial():
                    last_time_motor_disconected = rospy.get_time()
                    reset_error = True
                    continue
                if rospy.get_time() - last_time_motor_disconected > 3 and reset_error:
                    reset_error = False
                    if self.status_robot == "PAUSED" and self.mode_robot == "AUTO":
                        self.pub_mission_run_pause.publish(
                            StringStamped(stamp=rospy.Time.now(), data="RUN")
                        )
                #alive module check
                #
                # rospy.logerr("module died")
                self.count_alive = 0
                if self.mode_robot == "AUTO":
                    for module in self.module_list:
                        display_error = module.display_error
                        if not module.module_alive and not display_error:
                            rospy.logerr(f"Module {module.display_name} is not alive!")
                            self.count_alive = self.count_alive + 1
                if self.count_alive != 0:
                    if self.error_start_time is None:
                        self.error_start_time = rospy.Time.now()

                    # Nếu đã quá 1 giây và robot chưa vào ERROR
                    elapsed = rospy.Time.now() - self.error_start_time
                    if elapsed.to_sec() >= 3.0 and self.status_robot != "ERROR":
                        # rospy.loginfo("no problem")
                        self.module_error_stop = True
                        self.left_vel = 0.0
                        self.right_vel = 0.0
                        # Robot đã dừng sau 3 giây lỗi kéo dài
                else:
                    # Nếu hết lỗi → reset timer
                    self.error_start_time = None
                    self.module_error_stop = False
                # if rospy.get_time() - t < diff_time:
                #     # Must be sleep to prevent high CPU load
                #     rospy.sleep(NOP_TIME)
                #     continue
                # else:
                #     # rospy.loginfo("{}".format(round(01.0/(rospy.get_time() - t - NOP_TIME), 2)))
                #     t = rospy.get_time()
                # self.read_encoder_speed()
                if rospy.get_time() - last_error_check_time >= 1.0:
                    try:
                        self.read_error()
                        self.read_FM()
                    except Exception as e:
                        rospy.logerr_throttle(5.0, "Read error FF error: {}".format(e))
                    last_error_check_time = rospy.get_time()
                # self.read_close_loop_error()
                self.read_amps()
                # self.read_volate()
                # self.read_battery_amps()
                # self.check_motor_right_not_run()
                # self.check_motor_left_not_run()
                if self.use_encoder:
                    try:
                        self.read_encoder()
                    except Exception as e:
                        rospy.logerr_throttle(
                            5.0, "Read encoder error: {}".format(e)
                        )

                if (
                    rospy.get_time() - self.last_cmd_vel_time > self.cmd_vel_timeout
                    or self.keep_alive_timeout > 0.0
                    and rospy.get_time() - self.last_topic_alive
                    > self.keep_alive_timeout
                ):
                    self.left_vel = 0.0
                    self.right_vel = 0.0
                
                
                # Check FM Fault state
                if self.fm_fault_active or (rospy.get_time() - self.fm_fault_start_time < 1.0):
                     self.left_vel = 0.0
                     self.right_vel = 0.0
                
                # Resume mission 3 seconds after FM fault clears
                if self.fm_fault_paused and not self.fm_fault_active:
                    if rospy.get_time() - self.fm_fault_clear_time > 3.0:
                        self.fm_fault_paused = False
                        if self.status_robot == "PAUSED" and self.mode_robot == "AUTO":
                            self.pub_mission_run_pause.publish(
                                StringStamped(stamp=rospy.Time.now(), data="RUN")
                            )
                            rospy.loginfo("[FM RESUME] Mission resumed after 3 seconds")

                # if self.robot_status != None:
                #     if (
                #         self.is_safety
                #         or rospy.get_time() - self.last_safety_time > 0.5
                #     ) and self.robot_status["mode"] == "AUTO":
                #         safety_stop = True
                #         safety_vel.linear.z = rospy.get_time()
                #         self.safety_vel_pub.publish(safety_vel)
                #     else:
                #         safety_stop = False

                # if self.is_safety == True:
                #     self.left_vel = 0.0
                #     self.right_vel = 0.0
                #     safety_vel.linear.z = rospy.get_time()
                #     self.safety_vel_pub.publish(safety_vel)

                # Safety delay
                # if safety_stop == False and prev_safety_stop == True:
                #     last_safety_stop_time = rospy.get_time()
                # if safety_stop == False and rospy.get_time() - last_safety_stop_time < 2.0:
                #     self.left_vel = 0.0
                #     self.right_vel = 0.0

                # Force stop
                # if safety_stop == True and prev_safety_stop == False:
                #     begin_safety_stop = rospy.get_time()
                # if safety_stop == True and rospy.get_time() - begin_safety_stop > 0.5:
                #     self.left_vel = 0.0
                #     self.right_vel = 0.0
                # prev_safety_stop = safety_stop

                if (
                    self.use_encoder
                    and rospy.get_time() - self.last_encoder_received > 0.5
                ):
                    rospy.logerr_throttle(1, "Motor disconnected")

                # write cmd to dual motor
                if rospy.get_time() - self.time_motor_pre >= (
                    1.0 / self.frequency_motor
                ):
                    # try:
                    #     # print(rospy.get_time() - self.last_send_cmd_vel_time) # check time send speed command
                    #     if (
                    #         not self.disable_control_motor_right
                    #         and not self.disable_control_motor_left and self.emg_button and self.bumper
                    #     ):
                    #         self.control_motor(self.left_vel, self.right_vel)
                    try:
                        # print(rospy.get_time() - self.last_send_cmd_vel_time) # check time send speed command
                        if (
                            (self.disable_left and self.disable_right)
                            or self.disable_control_motor_right
                            or self.disable_control_motor_left
                        ):
                            self.control_motor(0, 0)
                        elif self.disable_left:
                            self.control_motor(0, self.right_vel)
                        elif self.disable_right:
                            self.control_motor(self.left_vel, 0)
                        else:
                            self.control_motor(self.left_vel, self.right_vel)
                    except Exception as e:
                        rospy.logerr_throttle(
                            5.0, "Send cmd_vel error: {}".format(e)
                        )
                    self.time_motor_pre = rospy.get_time()

                rate.sleep()
            except Exception as e:
                rospy.logerr_throttle(5.0, "Main error: {}".format(e))



def main():
    diff_driver = KeyaDiffDriver()


if __name__ == "__main__":
    main()
