#!/usr/bin/env bash

rosrun amr gen_release_file.py $1
rosrun control_system gen_release_file.py $1
rosrun mission_manager gen_release_file.py $1
rosrun slam_manager gen_release_file.py $1
rosrun agv_mongodb gen_release_file.py $1
rosrun moving_control gen_release_file.py $1
rosrun led_control gen_release_file.py $1
rosrun sound_control gen_release_file.py $1
rosrun arduino_ros gen_release_file.py $1
rosrun auto_docking gen_release_file.py $1
rosrun charging_process gen_release_file.py $1
rosrun clear_log gen_release_file.py $1
rosrun keya_servo gen_release_file.py $1
rosrun move_base_manager gen_release_file.py $1
rosrun system_setting gen_release_file.py $1
rosrun agv_common_library gen_release_file.py $1
rosrun scan_safety gen_release_file.py $1
