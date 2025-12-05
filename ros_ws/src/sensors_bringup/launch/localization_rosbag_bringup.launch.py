import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node 

def generate_launch_description():
    
    bringup_pkg = get_package_share_directory('sensors_bringup')

    config_file = os.path.join(
        bringup_pkg, 'config', 'config.yaml'
    )

    static_tf_node = Node(
        package='sensors_bringup',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        parameters=[config_file]
    )

    instance_seg_node = Node(
        package='sensors_processing',
        executable='instance_seg_node',
        name='instance_seg_node',
    )

    drone_localization_node = Node(
        package='sensors_processing',
        executable='drone_localization_node',
        name='drone_localization_node',
    )

    return LaunchDescription([
        LogInfo(msg="Starting static tf2..."),
        static_tf_node,
        LogInfo(msg="Starting sensor processing..."),
        instance_seg_node,
        drone_localization_node
    ])