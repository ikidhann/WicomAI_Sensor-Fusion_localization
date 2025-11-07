import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
import os

def generate_launch_description():
    
    bringup_pkg = get_package_share_directory('sensors_bringup')
    driver_pkg = get_package_share_directory('sensors_driver')
    processing_pkg = get_package_share_directory('sensors_processing')
    velodyne_pkg = get_package_share_directory('velodyne') 

    config_file = os.path.join(
        bringup_pkg, 'config', 'config.yaml'
    )
    configs = {'config_file': config_file}

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_pkg, 'launch', 'velodyne-all-nodes-VLP32C-launch.py')
        ),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_pkg, 'launch', 'camera_logitech.launch.py')
        ),
        launch_arguments=configs.items()
    )
    
    processing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(processing_pkg, 'launch', 'lidar_cam_sync.launch.py')
        )
    )

    return LaunchDescription([
        LogInfo(msg="Starting sensor drivers..."),
        velodyne_launch,
        camera_launch,
        LogInfo(msg="Starting static tf2..."),
        LogInfo(msg="Starting sensor processing..."),
        processing_launch
    ])