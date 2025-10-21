from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lidar_cam_sync'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wicomai',
    maintainer_email='wicomai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_cam_sync_node = lidar_cam_sync.lidar_cam_synchronizer:main',
            'cam_publisher_node = lidar_cam_sync.cam_publisher:main',
        ],
    },
)
