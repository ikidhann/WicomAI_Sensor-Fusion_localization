from pathlib import Path
from rosbags.rosbag2 import Reader
from sensor_msgs_py import point_cloud2
import numpy as np

def save_lidar_data(bag_file_path, output_dir, topic_name):
    bag_file = Path(bag_file_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with Reader(bag_file) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]
        for i, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=connections)):
            msg = point_cloud2.read_points(rawdata)
            points = np.array(list(msg))
            np.save(output_dir / f'frame_{i:06d}.npy', points)

if __name__ == "_main_":
    bag_file_path = '/home/wicomai/ros2_ws1/rosbag2_2025_10_15-23_32_17/rosbag2_2025_10_15-23_32_17_0.mcap'
    output_dir = '/home/wicomai/ros2_ws1/out_dir'
    topic_name = 'sync_pcd'
    save_lidar_data(bag_file_path, output_dir, topic_name)