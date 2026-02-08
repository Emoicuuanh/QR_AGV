import agf_mc_protocol
import yaml
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
from std_stamped_msgs.msg import (
    StringAction,
    StringStamped,
    StringResult,
    StringFeedback,
    StringGoal,
    Int8Stamped,
    EmptyStamped,
)
plc = agf_mc_protocol
from module_manager import ModuleServer, ModuleClient, ModuleStatus

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

class ElevatorAction(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        if not self.load_config():
            return 
        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            StringAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
                # Action client
        self.moving_control_client = actionlib.SimpleActionClient(
            "/moving_control", StringAction
        )
        self.moving_control_client.wait_for_server()
        rospy.on_shutdown(self.shutdown)

        self._asm = ModuleServer(name)
        self.init_server()
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        self.config_path = kwargs["config_path"]
        self.robot_config_file = kwargs["robot_config_file"]
        self.server_config_file = kwargs["robot_define"]
        self.use_tf2 = False
        self.tf_listener = tf.TransformListener()
        #
        self.last_moving_control_fb = rospy.get_time()
        self.moving_control_result = -1
        #
        self.moving_control_error_code = ""
        # Database
        db_address = rospy.get_param("/mongodb_address")
        print_debug(db_address)
        self.db = mongodb(db_address)
        self.emg_status = True
        self.liftup_finish = False
        self.detect_vrack = False
        self.liftdown_finish = False
        self.plc_address = rospy.get_param("~plc_address", "192.168.20.65")
        self.port = rospy.get_param("~plc_port", 2002)

        self.lift_msg = Int8Stamped()
        self.disable_qr_code_msg = Int8Stamped()
        self.std_io_msg = StringStamped()
        self.last_time_get_lift_up = rospy.get_time()
        self.last_time_get_lift_down = rospy.get_time()

        self.vel_move_base = 0.0

        self.elevator_name = None
        elevator_name_str = rospy.get_param("~elevator_name", "ELEVATOR_B")
        if elevator_name_str == "ELEVATOR_B":
            self.elevator_name = ElevatorName.ELEVATOR_B
        elif elevator_name_str == "ELEVATOR_A":
            self.elevator_name = ElevatorName.ELEVATOR_A
        # Add more elevator names here if necessary

        self.resetTimeoutError = False
        self.is_plc_connect_fail = False

        self.data_run = StringStamped()
        self.data_run.data = "RUN"
        self.mode_robot = ""

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def robot_status_cb(self, msg):
        robot_status = json.loads(msg.data)
        if "status" in robot_status:
            self.status_robot = robot_status["status"]
        if "mode" in robot_status:
            self.mode_robot = robot_status["mode"]

    def error_robot_to_path_cb(self, msg):
        self.error_position = msg.error_position
        self.error_angle = msg.error_angle

    def odom_cb(self, msg):
        self.pose_odom2robot = msg.pose.pose

    def standard_io_cb(self, msg):
        data = json.loads(msg.data)
        if "lift_max_sensor" in data:
            if data["lift_max_sensor"] and (
                rospy.get_time() - self.last_time_get_lift_up >= 2
            ):
                self.liftup_finish = True
            if not data["lift_max_sensor"]:
                self.last_time_get_lift_up = rospy.get_time()
                self.liftup_finish = False
        if "lift_min_sensor" in data:
            if data["lift_min_sensor"] and (
                rospy.get_time() - self.last_time_get_lift_down >= 2
            ):
                self.liftdown_finish = True
            if not data["lift_min_sensor"]:
                self.last_time_get_lift_down = rospy.get_time()
                self.liftdown_finish = False
        if "emg_button" in data:
            self.emg_status = data["emg_button"]
        if "start_1_button" in data:
            self.start_1 = data["start_1_button"]
        if "start_2_button" in data:
            self.start_2 = data["start_2_button"]
        if "stop_1_button" in data:
            self.stop_1 = data["stop_1_button"]
        if "stop_2_button" in data:
            self.stop_2 = data["stop_2_button"]
        # if "detect_vrack" in data: self.detect_vrack = data["detect_vrack"]
        self.detect_vrack = False 

    def moving_control_fb(self, msg):
        self.last_moving_control_fb = rospy.get_time()

    def moving_control_result_cb(self, msg):
        self.moving_control_result = msg.status.status
        # rospy.logerr(self.moving_control_result)

    def moving_control_module_status_cb(self, msg):
        try:
            self.moving_control_error_code = json.loads(msg.data)["error_code"]
        except Exception as e:
            rospy.logerr("moving_control_module_status_cb: {}".format(e))  


    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def dynamic_reconfig_movebase(self, vel_x, publish_safety):
        new_config = {
            "max_vel_x": vel_x,
            "max_vel_trans": vel_x,
            "publish_safety": publish_safety,
        }
        for i in range(3):
            self.client_reconfig_movebase.update_configuration(new_config)
            rospy.sleep(0.1)

    def read_data_elevator(self):
        while True:
            # try:
            if 1:
                if self.print_first:
                    rospy.loginfo("START THREAD MONITOR IO PLC")
                    self.print_first = False
                if self.start_thread:
                    self.read_value_x()
                    self.read_value_y()
                    self.read_value_w_input()
                    self.read_value_w_output()
                    self.pub_io()
            # except Exception as e:
            #     rospy.logerr(e)
            sleep(0.1)

    def load_config(self):
        try:
            # Server config
            if os.path.exists(self.server_config_file):
                with open(self.server_config_file) as file:
                    self.server_config = yaml.load(file, Loader=yaml.Loader)
                    rospy.loginfo("Server config file:")
                    rospy.loginfo(
                        yaml.dump(
                            self.server_config,
                            Dumper=YamlDumper,
                            default_flow_style=False,
                        )
                    )
                    if "server_address" not in self.server_config:
                        self.server_config = None
        except Exception as e:
            rospy.logerr("load_config: {}".format(e))
            return False
        return True

    def init_server(self):
        if self.server_config == None:
            rospy.loginfo_throttle(30, "Server was not configured!")
            return
        self.api_url = self.server_config["server_address"]
        self.data = self.server_config["agv_name"]  # {"agv":"AGV 01"}
        self.header = {
            "X-Parse-Application-Id": "APPLICATION_ID",
            "X-Parse-Master-Key": "YOUR_MASTER_KEY",
            "Content-Type": "application/json",
        }

    def upDateCart(self, type, name, cell, cart_no, lot_no):
        data = {
            "function": "UPDATE_CART",
            "type": type,
            "name": name,
            "cell": cell,
            "cart": cart_no,
            "lot": lot_no,
        }
        try:
            response = requests.post(
                self.api_url + "functions/agvapi",
                data=json.dumps(data),
                headers=self.header,
                timeout=1,
            )  # TOCHECK
            _temp = json.loads(response.text)
            print_debug(_temp)
            if _temp["result"] == "OK":
                return True
            else:
                return False
        except requests.exceptions.RequestException as e:
            rospy.logerr_throttle(10.0, e)
            return False

    def reset(self):
        rospy.logwarn("Start reset io")
        if plc.plc_connect_fail:
            rospy.logwarn("Reset io fail")
            return False
        else:
            counter_check = 0
            while True:
                plc.plc_connect_fail = False
                for i in x_value_address:
                    plc.write_x(i, [0])
                    rospy.sleep(0.01)
                for i in w_value_address_input:
                    plc.write_w(i, [0])
                    rospy.sleep(0.01)
                x_value = self.read_value_x()
                w_input_value = self.read_value_w_input()
                total_x = len(x_value_address)
                total_w_input = len(w_value_address_input)
                if (
                    w_input_value == [0] * total_w_input
                    and x_value == [0] * total_x
                    and not plc.plc_connect_fail
                ):
                    rospy.logwarn("Reset io success")
                    return True
                else:
                    counter_check += 1
                if counter_check > 3:
                    rospy.logwarn("Reset io fail")
                    return False
                rospy.sleep(1)

    def read_value_x(self):
        global pre_x_value
        index = 0
        for i in x_value_address:
            value = plc.read_x(i, 1)[0]
            rospy.sleep(0.01)
            if pre_x_value[index] != value:
                rospy.logwarn("Write data  X{} = {} to PLC".format(i, value))
                pre_x_value[index] = value
            index += 1
        return pre_x_value

    def read_value_y(self):
        global pre_y_value
        index = 0
        for i in y_value_address:
            value = plc.read_y(i, 1)[0]
            rospy.sleep(0.01)
            if pre_y_value[index] != value:
                rospy.logwarn("Read data Y{} = {} from PLC".format(i, value))
                pre_y_value[index] = value
            index += 1
        return pre_y_value

    def read_value_w_input(self):
        global pre_w_value_input
        index = 0
        for i in w_value_address_input:
            value = plc.read_w(i, 1)[0]
            rospy.sleep(0.01)
            if pre_w_value_input[index] != value:
                rospy.logwarn("Write data W{} = {} to PLC".format(i, value))
                pre_w_value_input[index] = value
            index += 1
        return pre_w_value_input

    def read_value_w_output(self):
        global pre_w_value_output
        index = 0
        for i in w_value_address_output:
            value = plc.read_w(i, 1)[0]
            rospy.sleep(0.01)
            if pre_w_value_output[index] != value:
                rospy.logwarn("Read data W{} = {} from PLC".format(i, value))
                pre_w_value_output[index] = value
            index += 1
        return pre_w_value_output

    def reset_value_all(self):
        global pre_x_value, pre_y_value, pre_w_value_input, pre_w_value_output
        pre_x_value = []
        pre_y_value = []
        pre_w_value_input = []
        pre_w_value_output = []
        for i in range(len(x_value_address)):
            pre_x_value.append(0)
        for i in range(len(y_value_address)):
            pre_y_value.append(0)
        for i in range(len(w_value_address_input)):
            pre_w_value_input.append(0)
        for i in range(len(w_value_address_output)):
            pre_w_value_output.append(0)

    def pub_io(self):
        sensors_msg_dict = {}
        for i in range(len(x_value_address)):
            sensors_msg_dict[str(x_value_address[i])] = pre_x_value[i]
        for i in range(len(y_value_address)):
            sensors_msg_dict[str(y_value_address[i])] = pre_y_value[i]
        for i in range(len(w_value_address_input)):
            sensors_msg_dict[str(w_value_address_input[i])] = pre_w_value_input[
                i
            ]
        for i in range(len(w_value_address_output)):
            sensors_msg_dict[str(w_value_address_output[i])] = (
                pre_w_value_output[i]
            )
        self.std_io_msg.stamp = rospy.Time.now()
        self.std_io_msg.data = json.dumps(sensors_msg_dict, indent=2)
        self.status_io_pub.publish(self.std_io_msg)

    def calculate_pose_offset(self, x_offset, cur_pose_x, cur_pose_y, cur_ori):
        x = cur_pose_x + x_offset * cos(cur_ori)
        y = cur_pose_y + x_offset * sin(cur_ori)
        p = Pose()
        p.position.x = x
        p.position.y = y
        q = quaternion_from_euler(0.0, 0.0, cur_ori)
        p.orientation = Quaternion(*q)
        return p

    def diff_angle(self, current_pose, target_pose):
        try:
            angel_target_pose = euler_from_quaternion(
                [
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w,
                ]
            )
        except:
            angel_target_pose = euler_from_quaternion(
                [
                    target_pose.pose.orientation.x,
                    target_pose.pose.orientation.y,
                    target_pose.pose.orientation.z,
                    target_pose.pose.orientation.w,
                ]
            )
        angel_target_pose = angel_target_pose[2]
        angle_current_pose = euler_from_quaternion(
            [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ]
        )
        angle_current_pose = angle_current_pose[2]
        diff_angle = angel_target_pose - angle_current_pose
        diff_angle = (diff_angle + pi) % (2 * pi) - pi
        diff_angle = degrees(diff_angle)
        return diff_angle

    def get_odom(self):
        self.pose_map2robot = Pose()
        if self.use_tf2:
            try:
                trans = self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    time=rospy.Time(0),
                    timeout=rospy.Duration(5),
                )
                self.trans = [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    0,
                ]
                self.rot = [
                    0,
                    0,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
                self.pose_map2robot.position.x = self.trans[0]
                self.pose_map2robot.position.y = self.trans[1]
                self.pose_map2robot.position.z = 0
                self.pose_map2robot.orientation.x = 0
                self.pose_map2robot.orientation.y = 0
                self.pose_map2robot.orientation.z = self.rot[2]
                self.pose_map2robot.orientation.w = self.rot[3]
                return True
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                KeyboardInterrupt,
            ):
                rospy.logwarn("TF exception ")
                return False
        else:
            try:
                self.tf_listener.waitForTransform(
                    "/map", "/base_link", rospy.Time(0), rospy.Duration(4.0)
                )
                # rospy.loginfo("transform found :)")
                self.trans, self.rot = self.tf_listener.lookupTransform(
                    "/map", "/base_link", rospy.Time(0)
                )
                ############self.rot is in quaternion############
                # print("CURRENT POSE :{}/n{}".format(self.trans, self.rot))
                self.pose_map2robot.position.x = self.trans[0]
                self.pose_map2robot.position.y = self.trans[1]
                self.pose_map2robot.position.z = 0
                self.pose_map2robot.orientation.x = 0
                self.pose_map2robot.orientation.y = 0
                self.pose_map2robot.orientation.z = self.rot[2]
                self.pose_map2robot.orientation.w = self.rot[3]
                return True
            except (
                tf.Exception,
                tf.ConnectivityException,
                tf.LookupException,
                KeyboardInterrupt,
            ):
                rospy.logwarn("TF exception")
                return False

    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def loop(self):
        r = rospy.Rate(1)
        status_msg = StringStamped()
        while not rospy.is_shutdown():
            if not self._asm.action_running:
                self._asm.module_status = ModuleStatus.WAITING
                self._asm.module_state = ModuleStatus.WAITING.toString()
                self._asm.error_code = ""
            else:
                self.send_feedback(self._as, self._asm.module_state)
            status_msg.stamp = rospy.Time.now()
            status_msg.data = json.dumps(
                {
                    "status": self._asm.module_status.toString(),
                    "state": self._asm.module_state,
                    "error_code": self._asm.error_code,
                }
            )
            self._asm.module_status_pub.publish(status_msg)
            r.sleep()

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
    rospy.init_node("elevator_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    ElevatorAction(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
