'''
Code for synchronizing colorimage, depthimage and pointcloud from Intel RealSense D455 camera and Velodyne LiDAR
====================================================================
Using ROS2
'''
#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.clock import Clock
from os.path import join as pathjoin
from sensor_msgs.msg import Image, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class TimeSyncNode(Node):

    def __init__(self):
        super().__init__('sync_node')
        self.color_pub = self.create_publisher(Image, '/sync/color', 10)
        #self.depth_pub = self.create_publisher(Image, '/sync_depth_image', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/sync_pcd', 10)
        self.color_sub = Subscriber(self, Image, '/camera/color')
        #self.depth_sub = Subscriber(self, Image, '/d455/depth')
        self.pcl_sub = Subscriber(self, PointCloud2, '/velodyne_points')

        queue_size = 10
        max_delay = 0.2
        #self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.pcl_sub], queue_size, max_delay, allow_headerless=True)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.pcl_sub], queue_size, max_delay, allow_headerless=True)
        self.sync.registerCallback(self.SyncCallback, self.get_clock().now().to_msg())

    def SyncCallback(self, color_img, pcd, now):
        color_img.header.stamp = now
        #depth.header.stamp = now
        pcd.header.stamp = now
        self.get_logger().info(f'Synch callback with {color_img.header.stamp} and {pcd.header.stamp} as times')
        self.color_pub.publish(color_img)
        #self.color_pub.publish(depth)
        self.pcd_pub.publish(pcd)

def main(args=None):

    rclpy.init(args=args)

    logger = rclpy.logging.get_logger('logger')
    logger.info('Starting the synchronization')

    time_sync = TimeSyncNode()
    rclpy.spin(time_sync)
    time_sync.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()