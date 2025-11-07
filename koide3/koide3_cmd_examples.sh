bag_path="$(pwd)/bags/bag_2511071534"
preprocessed_path="$(pwd)/bags_preprocessed"

# Preprocessing
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
  ros2 run direct_visual_lidar_calibration preprocess -d \
    --image_topic /sync/raw_image \
    --points_topic /sync/velodyne_points \
    --camera_info_topic /sync/cam_info \
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
  ros2 run direct_visual_lidar_calibration find_matches_superglue.py /tmp/preprocessed --superglue outdoor

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
  --device=/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration calibrate --background /tmp/preprocessed

# Result inspection
docker run \
  --rm \
  --net host \
  --gpus all \
  --device=/dev/dri \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3:jazzy \
  ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed