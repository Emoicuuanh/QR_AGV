import modbus_tcp_passbox
from pymodbus.client.sync import ModbusTcpClient
import yaml
import socket
import time

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
    PAUSED_BY_PASSBOX = 49
    COLLISION_POSSIBLE = 50
    EMG_PASSBOX = 60
    TIMEOUT_WHEN_WAIT_PASSBOX_ALLOW_MOVE = 70
    NETWORK_ERROR = 71
    REQUEST_OPEN_BARIE = 80


class MainStatePlace(EnumString):
    NONE = -1
    CHECK_PASSBOX_POSIBLE = 200
    REQUEST_ENTER_PASSBOX = 201
    CHECK_ENTER_POSSIBLE = 202
    CHECK_AGV_PLACE_COMPLETE = 203
    DONE_CARRY_IN = 204
    CARRY_IN_POSSIBLE = 205


class MainStatePick(EnumString):
    NONE = -1
    CHECK_PASSBOX_POSIBLE = 100
    REQUEST_ENTER_PASSBOX = 101
    CHECK_ENTER_POSSIBLE = 102
    CHECK_AGV_PICK_COMPLETE = 103
    DONE_CARRY_OUT = 104
    PAUSE = 105
######################
###PASS_BOX###
# INPUT PLC, OUTPUT AGV
open_dirty_side = 40200
open_clean_side = 40201
# OUTPUT PLC, INPUT AGV
done_open_dirty_side = 30200
done_open_clean_side = 30201
had_cart_in_hub =  30202
emg_passbox = 30203

######################
###Bộ nâng hạ###
# INPUT PLC, OUTPUT AGV
emg_agv_request = 40001
pick_agv_request = 40002
place_agv_request = 40003
close_barie_dirty_side = 40004
open_barie_dirty_side = 40005
agv_going_passbox = 40006
emg_agv = 40007
safety_off = 40008
# OUTPUT PLC, INPUT AGV
position_state = 30001 
place_or_pick_state = 30002
barie_state = 30003
error_state = 30004
light_curtain_state = 30005
error_code_1 = 30006
error_code_2 = 30007
mode_ban_nang_ha = 30008

class PassboxAction(object):
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

        #Publisher
        self.pub_lift_cmd = rospy.Publisher(
            "/lift_cart", Int8Stamped, queue_size=10
        )

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
        self.emg_status = True
        self.liftup_finish = False
        self.detect_vrack = False
        self.liftdown_finish = False
        self.plc_ip = rospy.get_param("~plc_ip", "192.168.3.250")
        self.plc_port = rospy.get_param("~plc_port", 502)
        rospy.loginfo("Connecting to PLC: {}:{}".format(self.plc_ip, self.plc_port))
        self.plc = modbus_tcp_passbox.ModbusTcpClient(self.plc_ip, self.plc_port,timeout=3.0)


        self.wareshare_ip = rospy.get_param("~wareshare_ip", "192.168.1.200")
        self.wareshare_port = rospy.get_param("~wareshare_port", 502)
        rospy.loginfo("Connecting to Wareshare: {}:{}".format(self.wareshare_ip, self.wareshare_port))
        self.wareshare = ModbusTcpClient(self.wareshare_ip, self.wareshare_port)

        self.lift_msg = Int8Stamped()
        self.disable_qr_code_msg = Int8Stamped()
        self.std_io_msg = StringStamped()
        self.last_time_get_lift_up = rospy.get_time()
        self.last_time_get_lift_down = rospy.get_time()

        self.vel_move_base = 0.0

        self.resetTimeoutError = False
        self.is_plc_connect_fail = False

        self.data_run = StringStamped()
        self.data_run.data = "RUN"
        self.mode_robot = ""

    def check_connected(self):
        return self.plc.is_connected() and self.wareshare.is_connected()
    
    def read_wareshare(self):
        self.wareshare = ModbusTcpClient(self.wareshare_ip, self.wareshare_port)
        self.wareshare.connect()
        rr = self.wareshare.read_discrete_inputs(0, 8, unit=1)
        if rr.isError():
            print("Read error:", rr)
            return None
        bits = [1 if b else 0 for b in rr.bits[:8]]
        return bits

    def shutdown(self):
        if self.check_connected():
            self.plc.close()
            self.wareshare.close()
        self.dynamic_reconfig_movebase(self.vel_move_base, True)
        self.moving_control_client.cancel_all_goals()

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
    ######## ##     ## ########  ######  ##     ## ######## ########
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ######      ###    ######   ##       ##     ##    ##    ######
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ######## ##     ## ########  ######   #######     ##    ########
    """

    def execute_cb(self, goal):
        while True:
            if self.get_odom():
                break
        use_server = True
        hub_type = "hub"
        try:
            # Hub config
            hub_cfg_file = os.path.join(self.config_path, hub_type + ".json")
            with open(hub_cfg_file) as j:
                hub_dict = json.load(j)
                dist_check_go_in = hub_dict["dist_check_go_in"]
                dist_check_go_out = hub_dict["dist_check_go_out"]
                max_error_position_in_hub = hub_dict[
                    "max_error_position_in_hub"
                ]
                max_error_angle_in_hub = hub_dict["max_error_angle_in_hub"]
                max_error_position_out_hub = hub_dict[
                    "max_error_position_out_hub"
                ]
                max_error_angle_out_hub = hub_dict["max_error_angle_out_hub"]
                safety_job_docking_forward = hub_dict[
                    "safety_job_docking_forward"
                ]
                safety_job_docking_backward = hub_dict[
                    "safety_job_docking_backward"
                ]
                safety_job_undocking_forward = hub_dict[
                    "safety_job_undocking_forward"
                ]
                safety_job_undocking_backward = hub_dict[
                    "safety_job_undocking_backward"
                ]
                footprint_dock = hub_dict["footprint_dock"]
                foorprint_undock = hub_dict["foorprint_undock"]
                enable_check_error_when_docking = hub_dict[
                    "enable_check_error_when_docking"
                ]
                distance_turn_off_safety_when_docking = hub_dict[
                    "distance_turn_off_safety_when_docking"
                ]
                vel_docking_hub = hub_dict["max_vel_docking"]
                if "safety_job_rotation" in hub_dict:
                    safety_job_rotation = hub_dict["safety_job_rotation"]
                else:
                    safety_job_rotation = "ROTATION"
            # Waiting path config
            waiting_path_cfg = os.path.join(
                self.config_path, "waiting_path.json"
            )
            with open(waiting_path_cfg) as j:
                waiting_path_dict = json.load(j)
                return_pose_dict = waiting_path_dict["waypoints"][0]["position"]
            # Docking path config
            docking_path_cfg = os.path.join(
                self.config_path, "docking_path.json"
            )
            with open(docking_path_cfg) as j:
                docking_path_dict = json.load(j)
            # UnDocking path config
            undocking_path_cfg = os.path.join(
                self.config_path, "undocking_path.json"
            )
            with open(undocking_path_cfg) as j:
                undocking_path_dict = json.load(j)
        except Exception as e:
            rospy.logerr("Read config file error: {}".format(e))
            self._as.set_aborted("Read config file error")
            return

        # Load data from action
        data_dict = json.loads(goal.data)
        rospy.logwarn(data_dict)
        direction = BACKWARD
        self.enable_safety = True
        try:
            hub_pose_x = data_dict["params"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["y"]
            waiting_pose_x = data_dict["params"]["waiting_position"]["x"]
            waiting_pose_y = data_dict["params"]["waiting_position"]["y"]
            lift_pose_x = data_dict["params"]["lift_position"]["x"]
            lift_pose_y = data_dict["params"]["lift_position"]["y"]
            self.type = "PASSBOX"
            self.name = data_dict["params"]["name"]
            self.cell = 0  # data_dict["params"]["cell"]
            self.cart = data_dict["params"]["cart"]
            self.lot = data_dict["params"]["lot"]
            if "properties" in data_dict["params"]:
                if "Safety" in data_dict["params"]["properties"]:
                    if data_dict["params"]["properties"]["Safety"] == "Disable":
                        self.enable_safety = False
                if "Invert" in data_dict["params"]["properties"]:
                    direction = data_dict["params"]["properties"]["invert"]

                if "Non_equal" in data_dict["params"]["properties"]:
                    self.non_equal = data_dict["params"]["properties"]["Non_equal"]
        except:
            hub_pose_x = data_dict["params"]["position"]["position"]["x"]
            hub_pose_y = data_dict["params"]["position"]["position"]["y"]
            waiting_pose_x = self.trans[0]
            waiting_pose_y = self.trans[1]
            self.type = "PASSBOX"
            self.name = "AGV 01"
            self.cell = 0
            self.cart = "VRACK"
            self.lot = "1"
            use_server = False
        if direction == FORWARD:
            cur_orient = atan2(
                hub_pose_y - lift_pose_y, hub_pose_x - lift_pose_x
            )
        else:
            cur_orient = atan2(
                lift_pose_y - hub_pose_y, lift_pose_x - hub_pose_x
            )

        # ============================================================
        # CALCULATE WAITING GOAL
        # ============================================================
        waiting_goal = StringGoal()
        waiting_pose = self.calculate_pose_offset(
            0,
            waiting_pose_x,
            waiting_pose_y,
            cur_orient,
        )
        waiting_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(waiting_pose, return_pose_dict)
        )
        waiting_goal.data = json.dumps(waiting_path_dict, indent=2)
        rospy.logwarn(
            "Waiting goal position:\n{}".format(
                json.dumps(waiting_path_dict, indent=2)
            )
        )

        # ============================================================
        # CALCULATE LIFT GOAL
        # ============================================================
        lift_goal = StringGoal()
        lift_pose = self.calculate_pose_offset(
            0,
            lift_pose_x,
            lift_pose_y,
            cur_orient,
        )
        lift_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(lift_pose, return_pose_dict)
        )
        lift_goal.data = json.dumps(lift_path_dict, indent=2)
        rospy.logwarn(
            "Lift goal position:\n{}".format(
                json.dumps(lift_path_dict, indent=2)
            )
        )

        # ============================================================
        # CALCULATE DOCKING GOAL
        # ============================================================
        docking_goal = StringGoal()
        docking_pose = self.calculate_pose_offset(
            0,
            hub_pose_x,
            hub_pose_y,
            cur_orient,
        )
        docking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(lift_pose, return_pose_dict)
        )
        docking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(docking_pose, return_pose_dict)
        )
        docking_goal.data = json.dumps(docking_path_dict, indent=2)
        rospy.logwarn(
            "Docking goal position:\n{}".format(
                (json.dumps(docking_path_dict, indent=2))
            )
        )

        # ============================================================
        # CALCULATE UNDOCKING GOAL
        # ============================================================
        undocking_goal = StringGoal()
        undocking_path_dict["waypoints"][0]["position"] = copy.deepcopy(
            obj_to_dict(docking_pose, return_pose_dict)
        )

        undocking_path_dict["waypoints"][1]["position"] = copy.deepcopy(
            obj_to_dict(lift_pose, return_pose_dict)
        )
        if "param_test" not in undocking_path_dict:
            undocking_path_dict["param_test"] = {}
        undocking_path_dict["param_test"][
            "need_to_wait_receive_new_path"
        ] = True
        undocking_goal.data = json.dumps(undocking_path_dict, indent=2)
        rospy.logwarn(
            "UnDocking goal position:\n{}".format(
                (json.dumps(undocking_path_dict, indent=2))
            )
        )

        pick_or_place = data_dict["params"]["pick_or_place"]
        if pick_or_place:
            goal_type = PICK
            _state = MainState.INIT
        else:
            goal_type = PLACE
            _state = MainState.WAIT_RESET_IO

        r = rospy.Rate(15)
        success = False
        _prev_state = MainState.NONE
        feedback_msg = ""
        self._asm.reset_flag()
        self._asm.action_running = True # kiểm tra server có còn hoạt động không
        self.pose_map2robot = None
        is_preemted = False # Kiểm tra xem server có bị preemted không
        first_check_timeout = True # Check timeout first time with plc
        _state_when_network_timeout = MainState.NONE # Lưu state tại thời điểm timeout trước khi set state mất kết nối plc
        if self.check_connected():
            rospy.sleep(1)
            rospy.logwarn("connect passbox success first time")
        else:
            rospy.logwarn("connect passbox false first time")
        while not rospy.is_shutdown():
            try:
                if not self.check_connected():
                    if first_check_timeout:
                        if (
                            _state == MainState.GOING_TO_OUT_OF_HUB
                            or _state == MainState.DOCKING_TO_HUB
                        ):
                            self.moving_control_run_pause_pub.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data="PAUSE"
                                )
                            )
                        _state_when_network_timeout = _state
                        first_check_timeout = False
                        rospy.sleep(0.5)
                    _state = MainState.NETWORK_ERROR
                    
            except Exception as e:
                rospy.logerr(e)

            if not self.get_odom():
                continue
            distance_to_hub = distance_two_points(
                self.pose_map2robot.position.x,
                self.pose_map2robot.position.y,
                hub_pose_x,
                hub_pose_y,
            )

            # ################################################################
            # ################################################################
            # ##                                                            ##
            # ##           PREEMPT LOGIC - TODO: IMPLEMENT LATER            ##
            # ##                                                            ##
            # ################################################################
            # ################################################################
            # if (
            #     self._as.is_preempt_requested()
            #     or self._asm.reset_action_req
            #     or is_preemted
            # ):
            #     # TODO: Add preempt handling logic here
            #     pass

            # ################################################################
            # ################################################################
            # ##                                                            ##
            # ##      CHECK MODULE STATUS & EMG - TODO: IMPLEMENT LATER     ##
            # ##      (Chỉ cần khi passbox có tín hiệu EMG từ PLC)         ##
            # ##                                                            ##
            # ################################################################
            # ################################################################
            # if self._asm.module_status != ModuleStatus.ERROR:
            #     self._asm.error_code = ""
            # if _state != MainState.PAUSED and self._asm.error_code == "":
            #     self._asm.module_status = ModuleStatus.RUNNING
            # TODO: Implement logic kiểm tra tín hiệu EMG từ passbox PLC nếu cần
            
            # """
            # .####.##....##.####.########
            # ..##..###...##..##.....##...
            # ..##..####..##..##.....##...
            # ..##..##.##.##..##.....##...
            # ..##..##..####..##.....##...
            # ..##..##...###..##.....##...
            # .####.##....##.####....##...
            # """
            # """
            # State:
            if _state == MainState.INIT:
                self.vel_move_base = rospy.get_param(
                    "/move_base/NeoLocalPlanner/max_vel_x"
                )
                self.dynamic_reconfig_movebase(vel_docking_hub, False)
                _state = MainState.SEND_GOTO_WAITING
            
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # ============================================================
            # State: WAIT_RESET_IO
            # ============================================================
            elif _state == MainState.WAIT_RESET_IO:
                _state = MainState.INIT         

            # """
            # ..######..########.##....##.########.........######...#######..########..#######.........##......##....###....####.########.####.##....##..######..
            # .##....##.##.......###...##.##.............##....##.##.....##....##....##.....##.........##..##..##...##.##....##.....##.....##..###...##.##....##.
            # .##.......##.......####..##.##.............##.......##.....##....##....##.....##.........##..##..##..##...##...##.....##.....##..####..##.##.......
            # ..######..######...##.##.##.##.....#######.##...####.##.....##....##....##.....##.........##..##..##.##.....##..##.....##.....##..##.##.##.##...####
            # .......##.##.......##..####.##.............##....##.##.....##....##....##.....##.........##..##..##.#########..##.....##.....##..##..####.##....##.
            # .##....##.##.......##...###.##.............##....##.##.....##....##....##.....##.........##..##..##.##.....##..##.....##.....##..##...###.##....##.
            # ..######..########.##....##.########........######...#######.....##.....#######..#######..###..###..##.....##.####....##....####.##....##..######..
            # """

            # State: SEND_GOTO_WAITING
            elif _state == MainState.SEND_GOTO_WAITING:
                self.moving_control_client.send_goal(
                    waiting_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.GOING_TO_WAITING
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # ..######....#######..........##......##....###....####.########.####.##....##..######..
            # .##....##..##.....##.........##..##..##...##.##....##.....##.....##..###...##.##....##.
            # .##........##.....##.........##..##..##..##...##...##.....##.....##..####..##.##.......
            # .##...####.##.....##.........##..##..##.##.....##..##.....##.....##..##.##.##.##...####
            # .##....##..##.....##.........##..##..##.#########..##.....##.....##..##..####.##....##.
            # .##....##..##.....##.........##..##..##.##.....##..##.....##.....##..##...###.##....##.
            # ..######....#######..#######..###..###..##.....##.####....##....####.##....##..######..
            # """
            # ============================================================
            # State: GOING_TO_WAITING
            # ============================================================
            elif _state == MainState.GOING_TO_WAITING:
                if self.enable_safety and first_go_to_waiting:
                    self.safety_job_name = safety_job_rotation
                else:
                    self.safety_job_name = ""
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    if goal_type == PICK:
                        _state = MainState.LIFT_MIN_FIRST
                    else:
                        _state = MainState.LIFT_MAX_FIRST
                # --------------------------------------------------------
                # Kiểm tra lỗi di chuyển
                # --------------------------------------------------------
                elif (
                    self.moving_control_result != GoalStatus.SUCCEEDED
                    and self.moving_control_result != GoalStatus.ACTIVE
                    and self.moving_control_result != -1
                ) or self.moving_control_error_code != "":
                    rospy.logerr(
                        "Go to waiting fail: {}".format(
                            GoalStatus.to_string(self.moving_control_result)
                        )
                    )  
                    _state = MainState.MOVING_ERROR
                    _state_bf_error = MainState.SEND_GOTO_WAITING
                    _state_when_error = _state        
                # --------------------------------------------------------
                # Kiểm tra timeout (mất kết nối với moving_control)
                # --------------------------------------------------------
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_GOTO_WAITING
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                # if is_pause_by_passbox:
                #     self._asm.reset_flag()
                #     self.moving_control_run_pause_pub.publish(
                #         StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                #     )
                #     _state_when_error = _state
                #     _state = MainState.PAUSED_BY_PASSBOX
                # else:
                #     if self._asm.pause_req:
                #         self._asm.reset_flag()
                #         self.moving_control_run_pause_pub.publish(
                #             StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                #         )
                #         _state_when_pause = _state
                #         _state = MainState.PAUSED

            # """
            # .##.......####.########.########.........##.....##.####.##....##.........########.####.########...######..########
            # .##........##..##..........##............###...###..##..###...##.........##........##..##.....##.##....##....##...
            # .##........##..##..........##............####.####..##..####..##.........##........##..##.....##.##..........##...
            # .##........##..######......##............##.###.##..##..##.##.##.........######....##..########...######.....##...
            # .##........##..##..........##............##.....##..##..##..####.........##........##..##...##.........##....##...
            # .##........##..##..........##............##.....##..##..##...###.........##........##..##....##..##....##....##...
            # .########.####.##..........##....#######.##.....##.####.##....##.#######.##.......####.##.....##..######.....##...
            # """


            # State: LIFT_MIN_FIRST
            elif _state == MainState.LIFT_MIN_FIRST:
                if self.liftdown_finish:
                    _state = MainStatePick.REQUEST_ENTER_PASSBOX
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            
            # """
            # .##.......####.########.########.........##.....##....###....##.....##.........########.####.########...######..########
            # .##........##..##..........##............###...###...##.##....##...##..........##........##..##.....##.##....##....##...
            # .##........##..##..........##............####.####..##...##....##.##...........##........##..##.....##.##..........##...
            # .##........##..######......##............##.###.##.##.....##....###............######....##..########...######.....##...
            # .##........##..##..........##............##.....##.#########...##.##...........##........##..##...##.........##....##...
            # .##........##..##..........##............##.....##.##.....##..##...##..........##........##..##....##..##....##....##...
            # .########.####.##..........##....#######.##.....##.##.....##.##.....##.#######.##.......####.##.....##..######.....##...
            # """

            # State: LIFT_MAX_FIRST

            elif _state == MainState.LIFT_MAX_FIRST:
                if self.liftup_finish:
                    _state = MainStatePlace.REQUEST_ENTER_PASSBOX
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_UP
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED


            # """
            # .########..########..#######..##.....##.########..######..########.........########.##....##.########.########.########.........########.....###.....######...######..########...#######..##.....##
            # .##.....##.##.......##.....##.##.....##.##.......##....##....##............##.......###...##....##....##.......##.....##....##.....##...##.##...##....##.##....##.##.....##.##.....##.##.....##
            # .##.....##.##.......##.....##.##.....##.##.......##..........##............##.......####..##....##....##.......##.....##....##.....##..##...##..##.......##.......##.....##.##.....##.##.....##
            # .########..######...##.....##.##.....##.######....######.....##............######...##.##.##....##....######...########.....########..##.....##..######...######..########..##.....##.##.....##
            # .##...##...##.......##..##.##.##.....##.##.............##....##............##.......##..####....##....##.......##...##......##.....##.#########.......##.......##.##.....##.##.....##..##...##.
            # .##....##..##.......##....##..##.....##.##.......##....##....##............##.......##...###....##....##.......##....##.....##.....##.##.....##.##....##.##....##.##.....##.##.....##...##.##..
            # .##.....##.########..#####.##..#######..########..######.....##....#######.########.##....##....##....########.##.....##....########..##.....##..######...######..########...#######.....###...
            # """

            # State: REQUEST_ENTER_PASSBOX (PLACE)
            elif _state == MainStatePlace.REQUEST_ENTER_PASSBOX:       
            # ################################################################
            # ################################################################
            # ##                                                            ##
            # ##   REQUEST_ENTER_PASSBOX (PLACE) - TODO: IMPLEMENT LATER    ##
            # ##                                                            ##
            # ################################################################
            # ################################################################
            # TODO: Implement logic giao tiếp với PLC passbox
                _state = MainStatePlace.CARRY_IN_POSSIBLE

            elif _state == MainStatePick.REQUEST_ENTER_PASSBOX:
            # ################################################################
            # ################################################################
            # ##                                                            ##
            # ##   REQUEST_ENTER_PASSBOX (PICK) - TODO: IMPLEMENT LATER     ##
            # ##                                                            ##
            # ################################################################
            # ################################################################
                _state = MainState.SEND_DOCKING_HUB
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED



            # """
            # ..######.....###....########..########..##....##.........####.##....##.........########...#######...######...######..####.########..##.......########
            # .##....##...##.##...##.....##.##.....##..##..##...........##..###...##.........##.....##.##.....##.##....##.##....##..##..##.....##.##.......##......
            # .##........##...##..##.....##.##.....##...####............##..####..##.........##.....##.##.....##.##.......##........##..##.....##.##.......##......
            # .##.......##.....##.########..########.....##.............##..##.##.##.........########..##.....##..######...######...##..########..##.......######..
            # .##.......#########.##...##...##...##......##.............##..##..####.........##........##.....##.......##.......##..##..##.....##.##.......##......
            # .##....##.##.....##.##....##..##....##.....##.............##..##...###.........##........##.....##.##....##.##....##..##..##.....##.##.......##......
            # ..######..##.....##.##.....##.##.....##....##....#######.####.##....##.#######.##.........#######...######...######..####.########..########.########
            # """

            # State: CARRY_IN_POSSIBLE (PLACE)
            elif _state == MainStatePlace.CARRY_IN_POSSIBLE:
            # ################################################################
            # ################################################################
            # ##                                                            ##
            # ##   CARRY_IN_POSSIBLE (PLACE) - TODO: IMPLEMENT LATER        ##
            # ##                                                            ##
            # ################################################################
            # ################################################################
                _state = MainState.SEND_DOCKING_HUB
                if self._asm.pause_req:
                    # self.moving_control_client.cancel_all_goals()
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED



            # """
            # ..######..########.##....##.########.........########...#######...######..##....##.####.##....##..######..........##.....##.##.....##.########.
            # .##....##.##.......###...##.##.....##.........##.....##.##.....##.##....##.##...##...##..###...##.##....##.........##.....##.##.....##.##.....##
            # .##.......##.......####..##.##.....##.........##.....##.##.....##.##.......##..##....##..####..##.##...............##.....##.##.....##.##.....##
            # ..######..######...##.##.##.##.....##.........##.....##.##.....##.##.......#####.....##..##.##.##.##...####.........#########.##.....##.########.
            # .......##.##.......##..####.##.....##.........##.....##.##.....##.##.......##..##....##..##..####.##....##..........##.....##.##.....##.##.....##
            # .##....##.##.......##...###.##.....##.........##.....##.##.....##.##....##.##...##...##..##...###.##....##..........##.....##.##.....##.##.....##
            # ..######..########.##....##.########..#######.########...#######...######..##....##.####.##....##..######...#######.##.....##..#######..########.
            # """

            # State: SEND_DOCKING_HUB
            elif _state == MainState.SEND_DOCKING_HUB:
                first_go_to_waiting = False
                check_go_in = True
                self.moving_control_client.send_goal(
                    docking_goal,
                    feedback_cb=self.moving_control_fb,
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.DOCKING_TO_HUB
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .########...#######...######..##....##.####.##....##..######..
            # .##.....##.##.....##.##....##.##...##...##..###...##.##....##.
            # .##.....##.##.....##.##.......##..##....##..####..##.##.......
            # .##.....##.##.....##.##.......#####.....##..##.##.##.##...####
            # .##.....##.##.....##.##.......##..##....##..##..####.##....##.
            # .##.....##.##.....##.##....##.##...##...##..##...###.##....##.
            # .########...#######...######..##....##.####.##....##..######..
            # """
            # State: DOCKING_TO_HUB
            elif _state == MainState.DOCKING_TO_HUB:
                if plc.read_y(autorator_enter_stop, 1)[0]:
                    self.disable_qr_code_msg.stamp = rospy.Time.now()
                    self.disable_qr_code_msg.data = 1
                    self.disable_check_error_qr_code_pub.publish(
                        self.disable_qr_code_msg
                    )
                    self.moving_control_client.cancel_all_goals()
                    rospy.sleep(2)
                    _state = MainState.SEND_GOTO_WAITING
                    is_pause_by_elevator = True
                    continue
                if direction == FORWARD:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_docking_forward
                    else:
                        self.safety_job_name = ""
                else:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_docking_backward
                    else:
                        self.safety_job_name = ""
                if distance_to_hub < distance_turn_off_safety_when_docking:
                    self.safety_job_name = ""
                if enable_check_error_when_docking:
                    if (
                        distance_to_hub > dist_check_go_in
                        and distance_to_hub < dist_check_go_in + 0.2
                    ):
                        if self.detect_vrack and goal_type == PLACE:
                            _state_bf_error = MainState.SEND_DOCKING_HUB
                            _state_when_error = _state
                            _state = MainState.UNABLE_PLACE_CART
                            self.moving_control_client.cancel_all_goals()
                            continue
                        disable_auto_get_center_tape = False

                        if (
                            abs(self.error_position)
                            >= max_error_position_out_hub
                        ):
                            if self.get_first_time_error:
                                self.get_first_time_error = False
                                first_time_error = rospy.get_time()
                            if (
                                rospy.get_time() - first_time_error > 0.5
                            ) or True:
                                self.get_first_time_error = True
                                _state_bf_error = MainState.SEND_DOCKING_HUB
                                _state_when_error = _state
                                _state = MainState.ALIGNMENT_SENSOR
                                self.step = 0
                                self._asm.reset_flag()
                                self.moving_control_run_pause_pub.publish(
                                    StringStamped(
                                        stamp=rospy.Time.now(),
                                        data="PAUSE",
                                    )
                                )
                                continue
                        else:
                            self.get_first_time_error = True
                            if abs(self.error_angle) >= max_error_angle_out_hub:
                                _state_bf_error = MainState.SEND_DOCKING_HUB
                                _state_when_error = _state
                                _state = MainState.ALIGNMENT_SENSOR
                                self.step = 0
                                self._asm.reset_flag()
                                self.moving_control_run_pause_pub.publish(
                                    StringStamped(
                                        stamp=rospy.Time.now(),
                                        data="PAUSE",
                                    )
                                )
                                continue
                    elif distance_to_hub <= dist_check_go_in:
                        disable_auto_get_center_tape = True
                        self.get_first_time_error = True
                        if (
                            abs(self.error_angle) >= max_error_angle_in_hub
                            or abs(self.error_position)
                            >= max_error_position_in_hub
                        ):
                            _state_bf_error = MainState.SEND_DOCKING_HUB
                            _state_when_error = _state
                            _state = MainState.COLLISION_POSSIBLE
                            self.moving_control_client.cancel_all_goals()
                            # self.moving_control_run_pause_pub.publish(
                            #     StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                            # )
                            continue
                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.CHECK_CART
                    continue
                elif (
                    self.moving_control_result != GoalStatus.SUCCEEDED
                    and self.moving_control_result != GoalStatus.ACTIVE
                    and self.moving_control_result != -1
                ) or self.moving_control_error_code != "":
                    rospy.logerr(
                        "Go to waiting fail: {}".format(
                            GoalStatus.to_string(self.moving_control_result)
                        )
                    )
                    _state_bf_error = MainState.SEND_GOTO_WAITING
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                    continue
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_DOCKING_HUB
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED
                    continue

            # """
            # ..######..##.....##.########..######..##....##..........######.....###....########..########
            # .##....##.##.....##.##.......##....##.##...##..........##....##...##.##...##.....##....##...
            # .##.......##.....##.##.......##.......##..##...........##........##...##..##.....##....##...
            # .##.......#########.######...##.......#####............##.......##.....##.########.....##...
            # .##.......##.....##.##.......##.......##..##...........##.......#########.##...##......##...
            # .##....##.##.....##.##.......##....##.##...##..........##....##.##.....##.##....##.....##...
            # ..######..##.....##.########..######..##....##.#######..######..##.....##.##.....##....##...
            # """
            # State: CHECK_CART
            elif _state == MainState.CHECK_CART:
                if goal_type == PICK:
                    _state = MainState.LIFT_MAX
                else:
                    _state = MainState.LIFT_MIN
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .##.......####.########.########.........##.....##....###....##.....##
            # .##........##..##..........##............###...###...##.##....##...##.
            # .##........##..##..........##............####.####..##...##....##.##..
            # .##........##..######......##............##.###.##.##.....##....###...
            # .##........##..##..........##............##.....##.#########...##.##..
            # .##........##..##..........##............##.....##.##.....##..##...##.
            # .########.####.##..........##....#######.##.....##.##.....##.##.....##
            # """
            # State: LIFT_MAX
            elif _state == MainState.LIFT_MAX:
                if self.liftup_finish:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, "", ""
                        ) and self.upDateCart(
                            "AGV", self.data, 0, self.cart, self.lot
                        ):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        rospy.sleep(1)
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    if self.lot == "":
                        self.db.saveStatusCartData(
                            "status_cart", "have_empty_cart"
                        )
                    else:
                        self.db.saveStatusCartData(
                            "status_cart", "have_full_cart"
                        )
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_UP
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .##.......####.########.########.........##.....##.####.##....##
            # .##........##..##..........##............###...###..##..###...##
            # .##........##..##..........##............####.####..##..####..##
            # .##........##..######......##............##.###.##..##..##.##.##
            # .##........##..##..........##............##.....##..##..##..####
            # .##........##..##..........##............##.....##..##..##...###
            # .########.####.##..........##....#######.##.....##.####.##....##
            # """
            # State: LIFT_MIN
            elif _state == MainState.LIFT_MIN:
                if self.liftdown_finish:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, self.cart, self.lot
                        ) and self.upDateCart("AGV", self.data, 0, "", ""):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        rospy.sleep(1)
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    self.db.saveStatusCartData("status_cart", "no_cart")
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED


            # """
            # .##.......####.########.########.........##.....##....###....##.....##
            # .##........##..##..........##............###...###...##.##....##...##.
            # .##........##..##..........##............####.####..##...##....##.##..
            # .##........##..######......##............##.###.##.##.....##....###...
            # .##........##..##..........##............##.....##.#########...##.##..
            # .##........##..##..........##............##.....##.##.....##..##...##.
            # .########.####.##..........##....#######.##.....##.##.....##.##.....##
            # """
            # State: LIFT_MAX
            elif _state == MainState.LIFT_MAX:
                if self.liftup_finish:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, "", ""
                        ) and self.upDateCart(
                            "AGV", self.data, 0, self.cart, self.lot
                        ):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        rospy.sleep(1)
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    if self.lot == "":
                        self.db.saveStatusCartData(
                            "status_cart", "have_empty_cart"
                        )
                    else:
                        self.db.saveStatusCartData(
                            "status_cart", "have_full_cart"
                        )
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_UP
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .##.......####.########.########.........##.....##.####.##....##
            # .##........##..##..........##............###...###..##..###...##
            # .##........##..##..........##............####.####..##..####..##
            # .##........##..######......##............##.###.##..##..##.##.##
            # .##........##..##..........##............##.....##..##..##..####
            # .##........##..##..........##............##.....##..##..##...###
            # .########.####.##..........##....#######.##.....##.####.##....##
            # """
            # State: LIFT_MIN
            elif _state == MainState.LIFT_MIN:
                if self.liftdown_finish:
                    if not FAKE_QR_CODE and self.server_config != None:
                        if self.upDateCart(
                            self.type, self.name, self.cell, self.cart, self.lot
                        ) and self.upDateCart("AGV", self.data, 0, "", ""):
                            rospy.sleep(1)
                            _state = MainState.SEND_GOTO_OUT_OF_HUB
                        else:
                            # _state = MainState.UPDATE_CART_ERROR
                            rospy.logwarn("UPDATE_CART_ERROR --> RETRY")
                    else:
                        rospy.sleep(1)
                        _state = MainState.SEND_GOTO_OUT_OF_HUB
                    self.db.saveStatusCartData("status_cart", "no_cart")
                else:
                    self.lift_msg.stamp = rospy.Time.now()
                    self.lift_msg.data = LIFT_DOWN
                    self.pub_lift_cmd.publish(self.lift_msg)
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # State: SEND_GOTO_OUT_OF_HUB
            elif _state == MainState.SEND_GOTO_OUT_OF_HUB:
                is_send_goal_out = True
                check_go_in = False
                self.moving_control_client.send_goal(
                    undocking_goal, feedback_cb=self.moving_control_fb
                )
                self.moving_control_result = -1
                self.last_moving_control_fb = rospy.get_time()
                _state = MainState.GOING_TO_OUT_OF_HUB
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED


            # ..######....#######...........#######..##.....##.########.........##.....##....###....########.########.##.....##....###....##....##
            # .##....##..##.....##.........##.....##.##.....##....##............###...###...##.##......##....##.......##.....##...##.##...###...##
            # .##........##.....##.........##.....##.##.....##....##............####.####..##...##.....##....##.......##.....##..##...##..####..##
            # .##...####.##.....##.........##.....##.##.....##....##............##.###.##.##.....##....##....######...#########.##.....##.##.##.##
            # .##....##..##.....##.........##.....##.##.....##....##............##.....##.#########....##....##.......##.....##.#########.##..####
            # .##....##..##.....##.........##.....##.##.....##....##............##.....##.##.....##....##....##.......##.....##.##.....##.##...###
            # ..######....#######..#######..#######...#######.....##....#######.##.....##.##.....##....##....########.##.....##.##.....##.##....##
            # State: GOING_TO_OUT_OF_HUB
            elif _state == MainState.GOING_TO_OUT_OF_HUB:
                if direction == FORWARD:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_undocking_backward
                    else:
                        self.safety_job_name = ""
                else:
                    if self.enable_safety:
                        self.safety_job_name = safety_job_undocking_forward
                    else:
                        self.safety_job_name = ""

                if enable_check_error_when_docking:
                    if abs(distance_to_hub) < dist_check_go_out:
                        disable_auto_get_center_tape = True
                        self.get_first_time_error = True
                        if (
                            abs(self.error_angle) >= max_error_angle_in_hub
                            or abs(self.error_position)
                            >= max_error_position_in_hub
                        ):
                            _state_bf_error = MainState.SEND_DOCKING_HUB
                            _state_when_error = _state
                            _state = MainState.COLLISION_POSSIBLE
                            self.moving_control_client.cancel_all_goals()
                            continue

                if self.moving_control_result == GoalStatus.SUCCEEDED:
                    _state = MainState.DONE
                elif (
                    self.moving_control_result != GoalStatus.SUCCEEDED
                    and self.moving_control_result != GoalStatus.ACTIVE
                    and self.moving_control_result != -1
                ) or self.moving_control_error_code != "":
                    rospy.logerr(
                        "Go to waiting fail: {}".format(
                            GoalStatus.to_string(self.moving_control_result)
                        )
                    )
                    _state_bf_error = MainState.SEND_GOTO_OUT_OF_HUB
                    _state_when_error = _state
                    _state = MainState.MOVING_ERROR
                if rospy.get_time() - self.last_moving_control_fb >= 5.0:
                    rospy.logerr("/moving control disconnected!")
                    self.send_feedback(
                        self._as, GoalStatus.to_string(GoalStatus.ABORTED)
                    )
                    _state_bf_error = MainState.SEND_GOTO_OUT_OF_HUB
                    _state_when_error = _state
                    _state = MainState.MOVING_DISCONNECTED
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    self.moving_control_run_pause_pub.publish(
                        StringStamped(stamp=rospy.Time.now(), data="PAUSE")
                    )
                    _state_when_pause = _state
                    _state = MainState.PAUSED

            # """
            # .########...#######..##....##.########
            # .##.....##.##.....##.###...##.##......
            # .##.....##.##.....##.####..##.##......
            # .##.....##.##.....##.##.##.##.######..
            # .##.....##.##.....##.##..####.##......
            # .##.....##.##.....##.##...###.##......
            # .########...#######..##....##.########
            # """
            # State: DONE
            elif _state == MainState.DONE:
                self.dynamic_reconfig_movebase(self.vel_move_base, True)
                if goal_type == PLACE:
                    # [1040, 1041, 1042]
                    plc.write_x(carry_in_completed[source_floor - 1], [1])
                    if (
                        plc.read_x(carry_in_completed[source_floor - 1], 1)[0]
                        != 1
                    ):
                        rospy.logerr(
                            "Checking for carry_in_completed x: {} to write to PLC".format(
                                carry_in_completed[source_floor - 1]
                            )
                        )
                    else:
                        if not plc.plc_connect_fail:
                            if (
                                plc.read_w(15, 1)[0] and plc.read_y(1004, 1)[0]
                            ) or self.resetTimeoutError:
                                self.resetTimeoutError = False
                                t = datetime.now()
                                _state = MainStatePlace.DONE_CARRY_IN
                            else:
                                rospy.logerr(
                                    "wait for read_w(15, 1) and plc.read_y(1004, 1)"
                                )
                                if begin_check_timeout:
                                    timeout_waiting_elevator_respond = (
                                        rospy.get_time()
                                    )
                                    begin_check_timeout = False
                                else:
                                    if (
                                        rospy.get_time()
                                        - timeout_waiting_elevator_respond
                                        > 30
                                    ):
                                        _state_when_error = _state
                                        _state = (
                                            MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE
                                        )
                elif goal_type == PICK:
                    # [1060, 1061, 1062]
                    plc.write_x(
                        carry_out_completion[destination_floor - 1], [1]
                    )
                    if (
                        plc.read_x(
                            carry_out_completion[destination_floor - 1], 1
                        )[0]
                        != 1
                    ):
                        rospy.logerr(
                            "Checking for carry_out_completion x: {} to write to PLC".format(
                                carry_out_completion[destination_floor - 1]
                            )
                        )
                    else:
                        # [1080, 1081, 1082]
                        if not plc.plc_connect_fail:
                            if (
                                plc.read_y(
                                    completion_ACK[destination_floor - 1], 1
                                )[0]
                                or self.resetTimeoutError
                            ):
                                self.resetTimeoutError = False
                                t = datetime.now()
                                _state = MainStatePick.DONE_CARRY_OUT
                            else:
                                rospy.logerr(
                                    "wait for read_y: {}".format(
                                        completion_ACK[destination_floor - 1]
                                    )
                                )
                                if begin_check_timeout:
                                    timeout_waiting_elevator_respond = (
                                        rospy.get_time()
                                    )
                                    begin_check_timeout = False
                                else:
                                    if (
                                        rospy.get_time()
                                        - timeout_waiting_elevator_respond
                                        > 30
                                    ):
                                        _state_when_error = _state
                                        _state = (
                                            MainState.TIMEOUT_WHEN_WAIT_ELEVATOR_ALLOW_MOVE
                                        )

            # """
            # .########...#######..##....##.########.........######.....###....########..########..##....##.........####.##....##
            # .##.....##.##.....##.###...##.##..............##....##...##.##...##.....##.##.....##..##..##...........##..###...##
            # .##.....##.##.....##.####..##.##..............##........##...##..##.....##.##.....##...####............##..####..##
            # .##.....##.##.....##.##.##.##.######..........##.......##.....##.########..########.....##.............##..##.##.##
            # .##.....##.##.....##.##..####.##..............##.......#########.##...##...##...##......##.............##..##..####
            # .##.....##.##.....##.##...###.##..............##....##.##.....##.##....##..##....##.....##.............##..##...###
            # .########...#######..##....##.########.........######..##.....##.##.....##.##.....##....##....#######.####.##....##
            # """
            # State: DONE_CARRY_IN (PLACE)
            elif _state == MainStatePlace.DONE_CARRY_IN:
                # [1040, 1041, 1042]
                plc.write_x(carry_in_completed[source_floor - 1], [0])
                rospy.sleep(0.5)
                # [100, 102, 104]
                plc.write_w(destination_ST_indication[source_floor - 1], [0])
                plc.write_w(
                    destination_ST_indication[source_floor - 1] + 1, [0]
                )
                # [120, 121, 122]
                plc.write_w(carry_in_ID[source_floor - 1], [0])
                # [1030, 1031, 1032]
                plc.write_x(carry_in_request[source_floor - 1], [0])
                # plc.write_x(request_door_open[source_floor - 1], [0])
                if (
                    plc.read_x(carry_in_completed[source_floor - 1], 1)[0] == 0
                    and plc.read_w(
                        destination_ST_indication[source_floor - 1], 1
                    )[0]
                    == 0
                    and plc.read_w(
                        destination_ST_indication[source_floor - 1] + 1, 1
                    )[0]
                    == 0
                    and plc.read_w(carry_in_ID[source_floor - 1], 1)[0] == 0
                    and plc.read_x(carry_in_request[source_floor - 1], 1)[0]
                    == 0
                ):
                    if not plc.plc_connect_fail:
                        success = True
                        break

            # """
            # .########...#######..##....##.########.........######.....###....########..########..##....##..........#######..##.....##.########
            # .##.....##.##.....##.###...##.##..............##....##...##.##...##.....##.##.....##..##..##..........##.....##.##.....##....##...
            # .##.....##.##.....##.####..##.##..............##........##...##..##.....##.##.....##...####...........##.....##.##.....##....##...
            # .##.....##.##.....##.##.##.##.######..........##.......##.....##.########..########.....##.............##.....##.##.....##....##...
            # .##.....##.##.....##.##..####.##..............##.......#########.##...##...##...##......##.............##.....##.##.....##....##...
            # .##.....##.##.....##.##...###.##..............##....##.##.....##.##....##..##....##.....##.............##.....##.##.....##....##...
            # .########...#######..##....##.########.........######..##.....##.##.....##.##.....##....##....#######..#######...#######.....##...
            # """
            # State: DONE_CARRY_OUT (PICK)
            elif _state == MainStatePick.DONE_CARRY_OUT:
                # [1050, 1051, 1052]
                plc.write_x(carry_out_request[destination_floor - 1], [0])
                # [1060, 1061, 1062]
                plc.write_x(carry_out_completion[destination_floor - 1], [0])
                if (
                    plc.read_x(carry_out_request[destination_floor - 1], 1)[0]
                    == 0
                    and plc.read_x(
                        carry_out_completion[destination_floor - 1], 1
                    )[0]
                    == 0
                ):
                    if not plc.plc_connect_fail:
                        success = True
                        break

        rospy.logwarn("Close connect to plc")
        if plc.plc_close():
            rospy.logwarn("Closed connect to plc when finish")
        else:
            rospy.logwarn("Close connect to plc when finish false")
        self._asm.action_running = False
        if success:
            self.start_thread = False
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)

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
    rospy.init_node("passbox_server", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    PassboxAction(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
