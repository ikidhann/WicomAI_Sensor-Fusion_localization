from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_cam_sync_node = Node(
        package='sensors_processing',
        executable='lidar_cam_sync_node',
        name='lidar_cam_sync_node',
    )

    return LaunchDescription([
        lidar_cam_sync_node,
    ])

