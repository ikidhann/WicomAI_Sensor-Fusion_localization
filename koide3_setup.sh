bag_path=$(realpath livox)
preprocessed_path=$(realpath livox_preprocessed)

# Preprocessing
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $bag_path:/tmp/input_bags \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:jazzy \
  ros2 run direct_visual_lidar_calibration preprocess -av /tmp/input_bags /tmp/preprocessed

# Initial guess
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:jazzy \
  ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed

# Fine registration
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:jazzy \
  ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed

# Result inspection
docker run \
  --rm \
  --net host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $preprocessed_path:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:jazzy \
  ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed