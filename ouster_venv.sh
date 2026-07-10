#!/bin/bash
source ros_ws_ouster/install/setup.bash
source ouster-venv/bin/activate
export PYTHONPATH="$(pwd)/ouster-venv/lib/python3.12/site-packages:$PYTHONPATH"

export rosbag_record_path="bags/bag_$(date +%y%m%d%H%M)"
alias ros_launch_ouster='ros2 launch ouster_ros driver.launch.py'
alias ros_launch_sensors_sync='ros2 launch sensors_bringup sensors_sync_bringup.launch.py'
alias ros_launch_projection='ros2 launch sensors_bringup projection_bringup.launch.py'
alias ros_launch_localization='ros2 launch sensors_bringup localization_bringup.launch.py'
alias ros_launch_localization_rosbag='ros2 launch sensors_bringup localization_rosbag_bringup.launch.py'
alias ros_launch_projection_rosbag='ros2 launch sensors_bringup projection_rosbag_bringup.launch.py'
alias ros_launch_extrinsic_tuner='ros2 launch sensors_bringup extrinsic_tuner_bringup.launch.py'
alias ros_record_sensors='ros2 bag record -o $rosbag_record_path /sync/ouster_points /sync/raw_image /sync/cam_info'
