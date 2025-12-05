from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('sensors_driver'),
        'config', 'ids.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_cfg,
        description='Path to the parameters file for the camera node.'
    )

    cam_publisher_node = Node(
        package='sensors_driver',
        executable='cam_publisher_node',
        name='cam_publisher_node',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_arg,
        cam_publisher_node,
    ])

