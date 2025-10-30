# koide3-calib
# 1. How to Setup the ROS2 Workspace

cd /Documents/sensor_fusion/ros2-drone-localization/calib_ws # --> ROS2 workspace 
colcon build # --> if needed

- go to byobu / tmux terminal, source for each terminal
source setup.sh 

- launch the lidar and cam node
ros_launch_velodyne
ros_launch_lidar_sync


# 2. ROS2 bag record
ros2 bag record -o "$rosbag_record_path" [topics: /sync/raw_image /sync/velodyne_points /camera/cam_info]


# 3. Find calib params, please refer to koide3_calib.sh

#setup the path variables (bag_path and preprocessed_path)
bag_path=/home/wicomai-cv/Documents/sensor_fusion/ros2-drone-localization/bags/bag_2510291931
preprocessed_path=/home/wicomai-cv/Documents/sensor_fusion/ros2-drone-localization/bags_preprocessed

#run koide3 preprocess
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $bag_path:/tmp/input_bags \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration preprocess -dv \
    --image_topic /sync/raw_image \
    --points_topic /sync/velodyne_points \
    --camera_info_topic /camera/cam_info \
    /tmp/input_bags /tmp/preprocessed

#run koide3 superglue matching
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration find_matches_superglue.py /tmp/preprocessed --superglue outdoor

#run koide3 initial auto guess
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration initial_guess_auto /tmp/preprocessed

#run koide3 calibration
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration calibrate --background /tmp/preprocessed 

