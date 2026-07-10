================================================================================
  ROS2 Ouster Workspace — Setup & Running Guide 
================================================================================

NETWORK INFO
  PC IP         : 192.168.6.100
  Ouster IP     : 192.168.6.11
  Ouster Model  : OS-1-128
  Serial Number : 122118000544
  Firmware      : v2.5.3

--------------------------------------------------------------------------------
STEP 1 — Build the workspace (first time or after code changes)
--------------------------------------------------------------------------------

  cd ~/Documents/sensor_fusion/ros2-drone-localization/ros_ws_ouster
  colcon build --packages-select sensors_bringup sensors_processing sensors_driver custom_interfaces ouster_ros ouster_sensor_msgs

--------------------------------------------------------------------------------
STEP 2 — Activate the Ouster environment
--------------------------------------------------------------------------------

  cd ~/Documents/sensor_fusion/ros2-drone-localization
  source ouster_venv.sh

  This script does:
    - source ros_ws_ouster/install/setup.bash
    - source ouster-venv/bin/activate
    - exports rosbag_record_path alias
    - sets all ros_launch_* and ros_record_sensors aliases

--------------------------------------------------------------------------------
STEP 3 — Launch synchronization (Ouster + Camera + Sync Node)
--------------------------------------------------------------------------------

  ros_launch_sensors_sync

  This starts:
    - ouster_ros driver (connects to 192.168.6.11, timestamp: TIME_FROM_ROS_TIME)
    - cam_publisher_node (Logitech camera)
    - lidar_cam_sync_node (ApproximateTimeSynchronizer, slop=0.1s)

  Published sync topics:
    /sync/ouster_points   — time-aligned point cloud (frame: os_lidar)
    /sync/raw_image       — time-aligned camera image
    /sync/cam_info        — time-aligned camera info

  Expected terminal log (confirms sync is working):
    [lidar_cam_sync_node]: [PCD-IMG] Sync time deltas: X.XX ms

--------------------------------------------------------------------------------
STEP 4 — Verify in RViz (separate terminal)
--------------------------------------------------------------------------------

  source ouster_venv.sh
  rviz2

  RViz settings:
    Fixed Frame    : os_sensor
    PointCloud2    : topic = /sync/ouster_points
    Image          : topic = /sync/raw_image

--------------------------------------------------------------------------------
STEP 5 — Record bag for calibration (separate terminal)
--------------------------------------------------------------------------------

  source ouster_venv.sh
  ros_record_sensors

  Records: /sync/ouster_points  /sync/raw_image  /sync/cam_info
  Saved to: bags/bag_<timestamp>/

  While recording: slowly move the sensor rig to capture varied viewpoints.
  Stop recording with Ctrl+C when done.

--------------------------------------------------------------------------------
NOTES
--------------------------------------------------------------------------------

  - driver_params.yaml key settings:
      sensor_hostname    : '192.168.6.11'
      timestamp_mode     : 'TIME_FROM_ROS_TIME'
      point_cloud_frame  : os_lidar
      lidar_frame        : os_lidar
      sensor_frame       : os_sensor
      use_system_default_qos : false  (BEST_EFFORT QoS)

  - lidar_cam_sync_node uses BEST_EFFORT QoS to match ouster driver output.

  - TF tree published by ouster driver:
      os_sensor --> os_lidar
      os_sensor --> os_imu

  - After calibration, update T_lidar_camera in:
      ros_ws_ouster/src/sensors_bringup/config/config.yaml

--------------------------------------------------------------------------------
COMMANDS — Main working directory
--------------------------------------------------------------------------------

  cd ~/Documents/sensor_fusion/ros2-drone-localization/

--------------------------------------------------------------------------------
COMMANDS — Setup ROS2 Ouster Workspace
--------------------------------------------------------------------------------

  # Build ouster workspace (first time or after code changes)
  cd ros_ws_ouster
  colcon build --packages-select sensors_bringup sensors_processing sensors_driver custom_interfaces ouster_ros ouster_sensor_msgs
  cd ..

  # Activate ouster environment (run from ros2-drone-localization/)
  source ouster_venv.sh

--------------------------------------------------------------------------------
COMMANDS — Collecting sensor data via ROS2
--------------------------------------------------------------------------------

  # Source environment and launch sync (Ouster + Camera + Sync Node)
  source ouster_venv.sh
  ros_launch_sensors_sync

  # Visualize synced frames in RViz (separate terminal)
  source ouster_venv.sh
  rviz2
  # In RViz: Fixed Frame = os_sensor
  #          PointCloud2  topic = /sync/ouster_points
  #          Image        topic = /sync/raw_image

  # Record topics for calibration (separate terminal, while sync is running)
  # Topic list:
  #   Point Cloud : /sync/ouster_points
  #   Image       : /sync/raw_image
  #   Camera Info : /sync/cam_info
  source ouster_venv.sh
  ros_record_sensors
  # While recording: slowly move the sensor rig for varied viewpoints
  # Stop with Ctrl+C — bag saved to bags/bag_<timestamp>/

#ROS Record
ros2 bag record -d 30 -o "$rosbag_record_path" /sync/ouster_points /sync/raw_image /sync/cam_info
--------------------------------------------------------------------------------
COMMANDS — Find Lidar-Camera extrinsic calibration (koide3)
--------------------------------------------------------------------------------

  # Set path variables
  bag_path="$(pwd)/bags/bag_<your_recorded_bag>"
  preprocessed_path="$(pwd)/bags_preprocessed"

  # Step 1: Preprocess bag
  docker run --rm --net host --gpus all \
    -e DISPLAY=$DISPLAY -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v $bag_path:/tmp/input_bags \
    -v $preprocessed_path:/tmp/preprocessed \
    koide3:jazzy \
    ros2 run direct_visual_lidar_calibration preprocess -d \
      --image_topic /sync/raw_image \
      --points_topic /sync/ouster_points \
      --camera_info_topic /sync/cam_info \
      /tmp/input_bags /tmp/preprocessed

  # Step 2: Initial guess (manual)
  docker run --rm --net host --gpus all --device=/dev/dri \
    -e DISPLAY=$DISPLAY -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v $preprocessed_path:/tmp/preprocessed \
    koide3:jazzy \
    ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed

  # Step 3: Fine registration
  docker run --rm --net host --gpus all --device=/dev/dri \
    -e DISPLAY=$DISPLAY -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v $preprocessed_path:/tmp/preprocessed \
    koide3:jazzy \
    ros2 run direct_visual_lidar_calibration calibrate --background /tmp/preprocessed

  # Calibration result [x, y, z, qx, qy, qz, qw] located at:
  #   bags_preprocessed/calib.json
  # Copy result into T_lidar_camera in:
  #   ros_ws_ouster/src/sensors_bringup/config/config.yaml

--------------------------------------------------------------------------------
COMMANDS — Visualize LiDAR-Camera Projection
--------------------------------------------------------------------------------
  source ouster_venv.sh
  ros_launch_extrinsic_tuner

  source ouster_venv.sh
  ros_launch_projection

  # In RViz: add Image topic /lidar_cam_proj

--------------------------------------------------------------------------------
COMMANDS — Visualize Drone Localization
--------------------------------------------------------------------------------

  source ouster_venv.sh
  ros_launch_localization

  # In RViz: add Image topic /drone/localized_pcd

================================================================================
<!-- - glfw error 65544: X11: Failed to open display :1 --->
xhost +local:root 

