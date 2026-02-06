# Copy ROS_ENTRY_REAL, ROS_ENTRY_SIMULATION to $HOME/robot_config/ros_config.sh
ros_master=192.168.XXX.XXX
export ROS_WORKSPACE=$HOME/XXX_WS
export ROS_ENTRY_REAL="roslaunch amr_app amr_startup.launch"
export ROS_ENTRY_SIMULATION="roslaunch amr_app amr_startup.launch simulation:=true use_rviz:=true use_joystick:=false"
export ROS_MASTER_URI=http://$ros_master:11311
export ROS_HOSTNAME=$ros_master

export TAPE_SS_PORT=/dev/ttyS0
export RFID_PORT=/dev/ttyS1
export MOTOR_PORT=/dev/ttyS2
export ESP_PORT=x-x
export ARDUINO_PORT=x-x