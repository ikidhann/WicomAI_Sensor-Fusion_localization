import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 

def generate_launch_description():
    
    bringup_pkg = get_package_share_directory('sensors_bringup')
    driver_pkg = get_package_share_directory('sensors_driver')
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

    static_tf_node = Node(
        package='sensors_bringup',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        parameters=[config_file]
    )

    lidar_cam_sync_node = Node(
        package='sensors_processing',
        executable='lidar_cam_sync_node',
        name='lidar_cam_sync_node',
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
        LogInfo(msg="Starting sensor drivers..."),
        velodyne_launch,
        camera_launch,
        LogInfo(msg="Starting static tf2..."),
        static_tf_node,
        LogInfo(msg="Starting sensor processing..."),
        lidar_cam_sync_node,
        instance_seg_node,
        drone_localization_node
    ])