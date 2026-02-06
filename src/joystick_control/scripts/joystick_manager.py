#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import json
from math import pi
from safety_msgs.msg import SafetyStatus
from std_stamped_msgs.msg import Int8Stamped, StringStamped, EmptyStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Twist,
    PoseStamped,
    PoseWithCovarianceStamped,
)
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

from mongodb import mongodb
from module_manager import ModuleServer, ModuleStatus
from common_function import (
    EnumString,
)

class MainState(EnumString):
    NONE = -1
    STOP_BY_SAFETY = 2
    SAFETY_OFF = 3
    NORMAL = 4


# ============================================================
#   ADDED: Detect controller name
# ============================================================
def detect_controller():
    """
    Detect the connected joystick/controller name.
    Tries multiple methods to identify the controller.
    """
    # Method 1: Try reading from /sys/class/input/js0/device/name
    try:
        if os.path.exists("/sys/class/input/js0/device/name"):
            with open("/sys/class/input/js0/device/name", "r") as f:
                name = f.read().strip()
                if name:
                    rospy.loginfo("[detect_controller] Found via sysfs: %s" % name)
                    return name
    except Exception as e:
        rospy.logwarn("[detect_controller] Failed to read from sysfs: %s" % str(e))

    # Method 2: Try parsing /proc/bus/input/devices
    try:
        if os.path.exists("/proc/bus/input/devices"):
            with open("/proc/bus/input/devices", "r") as f:
                content = f.read()
                # Look for js0 handler and get the Name line above it
                lines = content.split('\n')
                for i, line in enumerate(lines):
                    if 'js0' in line and 'Handlers' in line:
                        # Search backwards for the Name line
                        for j in range(i-1, max(0, i-10), -1):
                            if lines[j].startswith('N: Name='):
                                name = lines[j].split('N: Name=')[1].strip().strip('"')
                                if name:
                                    rospy.loginfo("[detect_controller] Found via /proc: %s" % name)
                                    return name
    except Exception as e:
        rospy.logwarn("[detect_controller] Failed to read from /proc: %s" % str(e))

    # Method 3: Check if device exists at all
    if not os.path.exists("/dev/input/js0"):
        rospy.logwarn("[detect_controller] No joystick device found at /dev/input/js0")
        return "No Device"

    rospy.logwarn("[detect_controller] Could not determine controller name")
    return "Unknown"


class joystickControl:
    def __init__(self):
        self.init_varialble()

        # ADDED: detect controller
        self.controller_name = detect_controller()
        self.controller_recheck_interval = rospy.get_param("~controller_recheck_interval", 1.0)  # seconds
        self.last_controller_check = rospy.get_time()
        rospy.loginfo("=" * 60)
        rospy.loginfo("Detected Controller: %s" % self.controller_name)
        rospy.loginfo("Recheck interval: %.1f seconds" % self.controller_recheck_interval)

        if "Xbox" in self.controller_name or "X-Box" in self.controller_name or "360 pad" in self.controller_name:
            rospy.loginfo("Using Xbox → F710 mapping")
            self.apply_mapping = self.map_xbox_to_f710
        else:
            rospy.loginfo("Using native joystick mapping (F710)")
            self.apply_mapping = self.map_identity
        rospy.loginfo("=" * 60)

        self.init_ros()
        self.poll()


    # ============================================================
    #   ADDED: Mapping functions
    # ============================================================
    def map_identity(self, joy):
        return joy

    def map_xbox_to_f710(self, joy):
        out = Joy()
        out.header = joy.header

        # Axes mapping
        out.axes = [
            joy.axes[0],  # LX
            joy.axes[1],  # LY
            joy.axes[3],  # RX
            joy.axes[4],  # RY
            joy.axes[6],  # dpad LR
            joy.axes[7],  # dpad UD
        ]

        # Convert trigger axes → buttons
        # Xbox triggers: -1.0 (not pressed) to 1.0 (fully pressed)
        # Convert to 0 (not pressed) or 1 (pressed when > 0)
        LT = 1 if joy.axes[2] < 0 else 0
        RT = 1 if joy.axes[5] < 0 else 0
        
        # Buttons mapping
        out.buttons = [
            joy.buttons[2],  # X
            joy.buttons[0],  # A
            joy.buttons[1],  # B
            joy.buttons[3],  # Y
            joy.buttons[4],  # LB
            joy.buttons[5],  # RB
            LT,              # LT
            RT,              # RT
            joy.buttons[6],  # back
            joy.buttons[7],  # start
            joy.buttons[9],  # L-stick
            joy.buttons[10], # R-stick
        ]

        return out

    def recheck_controller(self):
        """Periodically recheck controller and update mapping if changed"""
        current_time = rospy.get_time()
        if current_time - self.last_controller_check >= self.controller_recheck_interval:
            self.last_controller_check = current_time
            new_controller_name = detect_controller()

            if new_controller_name != self.controller_name:
                self.controller_name = new_controller_name
                rospy.logwarn("=" * 60)
                rospy.logwarn("CONTROLLER CHANGED!")
                rospy.logwarn("New controller: %s" % self.controller_name)
                rospy.logwarn("=" * 60)

                if "Xbox" in self.controller_name or "X-Box" in self.controller_name or "360 pad" in self.controller_name:
                    rospy.loginfo("Switching to Xbox → F710 mapping")
                    self.apply_mapping = self.map_xbox_to_f710
                else:
                    rospy.loginfo("Switching to native joystick mapping (F710)")
                    self.apply_mapping = self.map_identity


    def init_ros(self):
        self.mode_status_pub = rospy.Publisher(
            "~module_status", StringStamped, queue_size=10
        )
        self.safety_job_pub = rospy.Publisher(
            "/safety_job_joystick_name", StringStamped, queue_size=5
        )
        self.footprint_job_pub = rospy.Publisher(
            "/safety_footprint_name", StringStamped, queue_size=5
        )
        self.pub_vel = rospy.Publisher(
            self.vel_topic_output, Twist, queue_size=5
        )
        self.pub_joy = rospy.Publisher(self.joy_remap, Joy, queue_size=10)
        rospy.Subscriber(self.joy_origin, Joy, self.joy_cb)
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber("/safety_job_name", StringStamped, self.set_safety_cb)
        rospy.Subscriber(self.vel_topic_input, Twist, self.vel_cb)
        rospy.Subscriber('/safety_status', SafetyStatus, self.safety_status_cb)
        rospy.Subscriber('/robot_status', StringStamped, self.robot_status_cb)
        # ModuleServer
        self._asm = ModuleServer(rospy.get_name())

    def init_varialble(self):
        # Others
        self.button_manual_status = False
        self.charge_manual_status = False
        self.current_safety_job_name_auto_mode = ""
        self.current_safety_job_name_manual_mode = ""
        self.pre_safety_job_name = ""
        self.vel_x = 0
        self.vel_theta = 0
        self.robot_mode = "NONE"  # Track robot mode from /robot_status
        self.footprint_job = rospy.get_param("~footprint_job", "amr_run_alone")
        self.safety_job_forward = rospy.get_param("~safety_job_forward", "joystick_forward")
        self.safety_job_backward = rospy.get_param("~safety_job_backward", "joystick_backward")
        self.safety_job_rotation = rospy.get_param("~safety_job_rotation", "joystick_rotation")
        self.invert_direct_joystick = rospy.get_param("~invert_direct_joystick", True)
        self.joy_remap = rospy.get_param("~joy_remap", "joy")
        self.joy_origin = rospy.get_param("~joy_origin", "joy_origin")
        self.vel_topic_input = rospy.get_param("~vel_topic_input", "teleop_joy_cmd_vel")
        self.vel_topic_output = rospy.get_param("~vel_topic_output", "teleop_joy_cmd_vel_safety")
        # safety
        self.last_safety_stop = rospy.get_time()
        self.safety_fields = []
        self.safety_field_idx = 0
        self.is_safety_stop = False
        self.is_safety = False
        self.disable_safety = False
        # vel limit
        self.max_vel_move_straight = [0, 0.1, 0.2, 0.4]
        self.max_vel_move_rotation = [0, 0.1, 0.2, 0.4]
        self.max_field_restrict_vel = min(len(self.max_vel_move_straight), len(self.max_vel_move_rotation))
        # state module
        self.state = MainState.NORMAL
        self.prev_state = MainState.NONE

        self.last_time_pub = rospy.get_time()


    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def joy_cb(self, msg):
        # Block completely if in BUTTON_CONTROL mode
        if self.robot_mode == "BUTTON_CONTROL":
            return  # Do nothing, let node sleep

        # Apply the appropriate mapping (xbox or identity)
        msg = self.apply_mapping(msg)

        joy_data = msg
        try:
            new_axis = []
            new_buttons = []

            # If in manual/button mode or manual charging: completely block joystick
            if self.charge_manual_status or self.button_manual_status:
                # In manual/button mode: joystick cannot affect anything including safety
                # Zero all axes and buttons
                joy_data.axes = [0.0] * len(joy_data.axes)
                joy_data.buttons = [0] * len(joy_data.buttons)
                self.pub_joy.publish(joy_data)

                # Publish zero Twist to stop any motion
                try:
                    self.pub_vel.publish(Twist())
                except Exception:
                    pass
            else:
                # Not in manual mode: process joystick normally
                if joy_data.buttons[2]:
                    self.disable_safety = True
                else:
                    self.disable_safety = False

                # Forward joystick data normally
                self.pub_joy.publish(joy_data)

        except rospy.ServiceException as e:
            pass

    def safety_status_cb(self, msg):
        self.safety_fields = list(msg.fields)  # [1, 2, 3, ...]
        self.is_safety_stop = False
        self.is_safety = False
        if len(self.safety_fields) and self.safety_fields[0] == 1:
            self.is_safety_stop = True
            self.last_safety_stop = rospy.get_time()
        else:
            self.is_safety_stop = False
        # print_warn("Update safety fields: {}".format(self.safety_fields))
        if len(self.safety_fields):
            for field_idx in range(len(self.safety_fields)):
                if self.safety_fields[field_idx] == 1:
                    self.is_safety = True
                    self.safety_field_idx = field_idx
                    break

    def set_safety_cb(self, msg):
        self.current_safety_job_name_auto_mode = msg.data

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "auto_manual_sw" in data: self.button_manual_status = data["auto_manual_sw"]
        if "manual_charging_detect" in data: self.charge_manual_status = data["manual_charging_detect"]

    def robot_status_cb(self, msg):
        """Callback to track robot mode from control_system"""
        try:
            status_dict = json.loads(msg.data)
            if "mode" in status_dict:
                self.robot_mode = status_dict["mode"]
        except Exception as e:
            rospy.logerr_throttle(5.0, "robot_status_cb error: {}".format(e))

    def vel_cb(self, msg):
        _state = MainState.NORMAL
        if self.invert_direct_joystick:
            msg.linear.x = -msg.linear.x
        self.vel_remap = msg
        self.vel_x = msg.linear.x
        self.vel_theta = msg.angular.z
        if not self.disable_safety:
            if self.is_safety and self.safety_field_idx <= self.max_field_restrict_vel:
                if self.vel_x > 0:
                    self.vel_remap.linear.x = min(self.vel_x, self.max_vel_move_straight[self.safety_field_idx])
                elif self.vel_x < 0:
                    self.vel_remap.linear.x = max(self.vel_x, -self.max_vel_move_straight[self.safety_field_idx])
                if self.vel_theta > 0:
                    self.vel_remap.angular.z = min(self.vel_theta, self.max_vel_move_rotation[self.safety_field_idx])
                elif self.vel_x < 0:
                    self.vel_remap.angular.z = max(self.vel_theta, -self.max_vel_move_rotation[self.safety_field_idx])
                if self.is_safety_stop or (rospy.get_time() - self.last_safety_stop) < 1:
                    _state = MainState.STOP_BY_SAFETY
                    self.pub_vel.publish(Twist())
                else:
                    self.pub_vel.publish(self.vel_remap)
            else:
                self.pub_vel.publish(self.vel_remap)
        else:
            self.pub_vel.publish(self.vel_remap)
            _state = MainState.SAFETY_OFF
        self.state = _state

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def manager_safety(self):
        if self.button_manual_status:
            if self.current_safety_job_name_auto_mode != self.pre_safety_job_name:
                self.pre_safety_job_name = self.current_safety_job_name_auto_mode
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.current_safety_job_name_auto_mode
                self.safety_job_pub.publish(msg)
        else:
            if rospy.get_time() - self.last_time_pub >= 2:
                self.last_time_pub = rospy.get_time()
                footprint_msg = StringStamped()
                footprint_msg.stamp = rospy.Time.now()
                footprint_msg.data = self.footprint_job
                self.footprint_job_pub.publish(footprint_msg)
            if self.vel_x > 0:
                self.pre_safety_job_name = self.safety_job_forward
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.safety_job_forward
                self.safety_job_pub.publish(msg)
            elif self.vel_x < 0:
                self.pre_safety_job_name = self.safety_job_backward
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.safety_job_backward
                self.safety_job_pub.publish(msg)
            else:
                if self.vel_theta != 0:
                    self.pre_safety_job_name = self.safety_job_rotation
                    msg = StringStamped()
                    msg.stamp = rospy.Time.now()
                    msg.data = self.safety_job_rotation
                    self.safety_job_pub.publish(msg)


    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def poll(self):
        r = rospy.Rate(10.0)
        status_msg = StringStamped()
        status_msg_dict = {
            "state": MainState.NORMAL.toString(),
        }
        while not rospy.is_shutdown():
            # Periodically recheck controller
            self.recheck_controller()

            self.manager_safety()
            if self.prev_state != self.state:
                rospy.loginfo(
                    "Loop state: {} -> {}".format(
                        self.prev_state.toString(), self.state.toString()
                    )
                )
                self.prev_state = self.state
            status_msg_dict = {
                "state": self.state.toString(),
            }
            status_msg.data = json.dumps(status_msg_dict, indent=2)
            status_msg.stamp = rospy.Time.now()
            self._asm.module_status_pub.publish(status_msg)
            self.state = MainState.NORMAL
            r.sleep()


def main():
    rospy.init_node("joystick_manager")
    rospy.loginfo("Init node: " + rospy.get_name())
    joystickControl()


if __name__ == "__main__":
    main()
