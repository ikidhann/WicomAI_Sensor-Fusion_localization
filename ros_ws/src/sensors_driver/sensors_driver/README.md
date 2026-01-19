*** Ros2 Pkg Installing for Radar ***                                                                 
sudo apt-get install ros-jazzy-radar-msgs                                                
sudo apt-get install ros-jazzy-can-msgs                                                 
sudo apt-get install ros-jazzy-ros2-socketcan                                            
sudo apt install ros-jazzy-pcl-ros  

*** Ros2 run ***
> ros2 run sensors_processing radar_publish_pcd_node
>ros2 run tf2_ros static_transform_publisher \ 0 0 0 0 0 0 \ map radar_link 
>rviz2
>ros2 topic echo /mr76/points_buffered
>-ros2 topic echo /clicked_point


#update


