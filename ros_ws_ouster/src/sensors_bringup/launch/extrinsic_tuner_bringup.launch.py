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

    extrinsic_tuner_node = Node(
        package='sensors_processing',
        executable='extrinsic_tuner_node',
        name='extrinsic_tuner_node',
    )

    return LaunchDescription([
        LogInfo(msg="Starting static tf2..."),
        static_tf_node,
        LogInfo(msg="Starting sensor processing..."),
        extrinsic_tuner_node
    ])