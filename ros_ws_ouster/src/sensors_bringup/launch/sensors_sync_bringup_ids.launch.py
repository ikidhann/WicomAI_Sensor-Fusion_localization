import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bringup_pkg = get_package_share_directory('sensors_bringup')
    driver_pkg = get_package_share_directory('sensors_driver')
    ouster_ros_pkg = get_package_share_directory('ouster_ros')

    # TODO: set up new config file
    config_file = os.path.join(
        bringup_pkg, 'config', 'new_config.yaml'
    )
    configs = {'config_file': config_file}

    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ouster_ros_pkg, 'launch', 'driver.launch.py')
        ),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_pkg, 'launch', 'new_camera_ids.launch.py')
        ),
        launch_arguments=configs.items()
    )

    # TODO make new lidar-ids cam sync node (to change this node)
    lidar_cam_sync_node = Node(
        package='sensors_processing',
        executable='lidar_cam_sync_node',
        name='lidar_cam_sync_node',
    )

    return LaunchDescription([
        LogInfo(msg="Starting sensor drivers..."),
        ouster_launch,
        camera_launch,
        LogInfo(msg="Starting static tf2..."),
        LogInfo(msg="Starting sensor processing..."),
        lidar_cam_sync_node
    ])
