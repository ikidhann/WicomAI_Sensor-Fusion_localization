from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    lidar_cam_sync_cfg = os.path.join(
        get_package_share_directory('lidar_cam_sync'),
        'config',
        'config.yaml'
    )

    cam_publisher_node = Node(
        package='lidar_cam_sync',
        executable='cam_publisher_node',
        name='cam_publisher_node',
        parameters=[lidar_cam_sync_cfg]
    )

    lidar_cam_sync_node = Node(
        package='lidar_cam_sync',
        executable='lidar_cam_sync_node',
        name='lidar_cam_sync_node',
    )

    return LaunchDescription([
        cam_publisher_node,
        lidar_cam_sync_node
    ])

