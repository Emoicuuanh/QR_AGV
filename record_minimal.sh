#!/bin/bash

# Minimal ROS bag recording script (smaller file size)
# Records only essential topics for basic debugging

# Get output filename
if [ -z "$1" ]; then
    OUTPUT_NAME="minimal_$(date +%Y%m%d_%H%M%S)"
else
    OUTPUT_NAME="$1"
fi

BAGS_DIR="$HOME/rosbags"
mkdir -p "$BAGS_DIR"
OUTPUT_PATH="$BAGS_DIR/${OUTPUT_NAME}.bag"

echo "=========================================="
echo "Recording Minimal ROS Bag"
echo "=========================================="
echo "Output: $OUTPUT_PATH"
echo "Press Ctrl+C to stop recording"
echo "=========================================="

# Record only essential topics
rosbag record -O "$OUTPUT_PATH" \
    /odom \
    /cmd_vel \
    /data_gls621 \
    /safety_status \
    /error_robot_to_path \
    /robot_state_direction

echo ""
echo "Recording stopped. Bag saved to: $OUTPUT_PATH"
