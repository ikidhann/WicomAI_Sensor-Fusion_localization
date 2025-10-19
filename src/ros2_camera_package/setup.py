from setuptools import find_packages, setup

package_name = 'ros2_camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wicomai',
    maintainer_email='krishna603@kookmin.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd455_color = ros2_camera_package.ros2_camera_publisher:main',
            'subscriber = ros2_camera_package.ros2_image_subscriber:main',
            'lidar_sync = ros2_camera_package.sync_lidar:main'
        ],
    },
)
