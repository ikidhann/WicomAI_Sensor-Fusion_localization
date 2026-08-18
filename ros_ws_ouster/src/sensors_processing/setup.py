from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sensors_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model'), glob('model/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wicomai-cv',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_cam_sync_node = sensors_processing.lidar_cam_sync:main',
            'lidar_cam_proj_node = sensors_processing.lidar_cam_proj:main',
            'drone_localization_node = sensors_processing.drone_localization:main',
            'instance_seg_node = sensors_processing.segmentation:main',
            'extrinsic_tuner_node = sensors_processing.extrinsic_tuner:main',
            'logi_cam_localization_node = sensors_processing.logi_cam_localization:main',
            'radar_localization_node = sensors_processing.radar_localization:main',
            'lidar_cam_ids_sync_node = sensors_processing.lidar_cam_ids_sync:main',
        ],
    },
)
