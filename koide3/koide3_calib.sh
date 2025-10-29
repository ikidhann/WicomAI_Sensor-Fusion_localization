bag_path=/home/wicomai-cv/Documents/sensor_fusion/ros2-drone-localization/bags/bag_2510291931
preprocessed_path=/home/wicomai-cv/Documents/sensor_fusion/ros2-drone-localization/bags_preprocessed
docker build -t koide3:jazzy -f docker/jazzy/Dockerfile_with_superglue .

# Preprocessing
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $bag_path:/tmp/input_bags \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration preprocess -d \
    --image_topic /sync/raw_image \
    --points_topic /sync/velodyne_points \
    --camera_info_topic /camera/cam_info \
    /tmp/input_bags /tmp/preprocessed

# Initial guess
# docker run \
#   --rm \
#   --net host \
#   --gpus all \
#   -e DISPLAY=$DISPLAY \
#   -v $HOME/.Xauthority:/root/.Xauthority \
#   -v $preprocessed_path:/tmp/preprocessed \
#   koide3:jazzy \
#   ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed

docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration find_matches_superglue.py /tmp/preprocessed --superglue indoor

docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration initial_guess_auto /tmp/preprocessed

# Fine registration
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed

# Result inspection
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed