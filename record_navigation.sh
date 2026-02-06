#!/bin/bash

# Script to record ROS bag for NeoLocalPlanner debugging
# Usage: ./record_navigation.sh [output_name]

# Get output filename (default: navigation_YYYYMMDD_HHMMSS)
if [ -z "$1" ]; then
    OUTPUT_NAME="navigation_$(date +%Y%m%d_%H%M%S)"
else
    OUTPUT_NAME="$1"
fi

# Create bags directory if it doesn't exist
BAGS_DIR="$HOME/rosbags"
mkdir -p "$BAGS_DIR"

OUTPUT_PATH="$BAGS_DIR/${OUTPUT_NAME}.bag"

echo "=========================================="
echo "Recording ROS Bag for NeoLocalPlanner"
echo "=========================================="
echo "Output: $OUTPUT_PATH"
echo "Press Ctrl+C to stop recording"
echo "=========================================="

# Record important topics for NeoLocalPlanner
rosbag record -O "$OUTPUT_PATH" \
    /odom \
    /cmd_vel \
    /move_base/local_costmap/costmap \
    /move_base/global_costmap/costmap \
    /move_base/NeoLocalPlannerROS/local_plan \
    /move_base/NeoLocalPlannerROS/global_plan \
    /move_base/current_goal \
    /move_base/goal \
    /move_base/status \
    /data_gls621 \
    /safety_status \
    /safety_camera_status \
    /run_pause_req \
    /run_pause_req_by_traffic \
    /error_robot_to_path \
    /robot_state_direction \
    /safety_job_name \
    /safety_footprint_name \
    /lost_goal_label \
    /tf \
    /tf_static

echo ""
echo "Recording stopped. Bag saved to: $OUTPUT_PATH"
echo "To play back: rosbag play $OUTPUT_PATH"
echo "To get info: rosbag info $OUTPUT_PATH"
