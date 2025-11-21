#!/bin/bash
source ros_ws/install/setup.sh
source calib-venv/bin/activate
export PYTHONPATH="$(pwd)/calib-venv/lib/python3.12/site-packages:$PYTHONPATH"

export rosbag_record_path="../bags/bag_$(date +%y%m%d%H%M)"
alias ros_launch_velodyne='ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py'
alias ros_launch_sensors_sync='ros2 launch sensors_bringup sensors_sync_bringup.launch.py'
alias ros_launch_projection='ros2 launch sensors_bringup projection_bringup.launch.py'
alias ros_launch_localization='ros2 launch sensors_bringup localization_bringup.launch.py'
alias ros_launch_localization_rosbag='ros2 launch sensors_bringup localization_rosbag_bringup.launch.py'

