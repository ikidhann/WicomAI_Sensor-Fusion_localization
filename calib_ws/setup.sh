source ../calib-venv/bin/activate
source install/setup.bash

export rosbag_record_path="../bags/bag_$(date +%y%m%d%H%M)"
alias ros_launch_velodyne='ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py'
alias ros_launch_lidar_proj='ros2 launch lidar_cam_sync lidar_cam_sync-launch.py'

