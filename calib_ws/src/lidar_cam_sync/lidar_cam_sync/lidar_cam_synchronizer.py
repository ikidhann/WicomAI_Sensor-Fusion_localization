import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import message_filters


class LidarCameraSynchronizer(Node):
    def __init__(self):
        super().__init__('lidar_camera_sync_node')
        self.lidar_sub_ = message_filters.Subscriber(self, PointCloud2,'/velodyne_points')

        self.cam_sub_ = message_filters.Subscriber(self, Image, '/camera/raw_image') # TODO: check this

        # Publishers for viewing synchronized messages
        self.sync_raw_img_pub_ = self.create_publisher(Image, "/sync/raw_image", 10)
        self.sync_pcd_pub_ = self.create_publisher(PointCloud2, "/sync/velodyne_points", 10)

        self.ts_ = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub_, self.cam_sub_],
            queue_size=10,
            slop=0.1, # seconds
            allow_headerless=True
        )

        self.ts_.registerCallback(self.synchronized_callback)
        self.get_logger().info("Lidar-Camera Sync Node has been started.")
    
    
    def synchronized_callback(self, pcd_msg:PointCloud2, raw_image_msg:Image):

        pcd_msg.header.stamp = self.get_clock().now().to_msg()
        raw_image_msg.header.stamp =  self.get_clock().now().to_msg()

        self.sync_raw_img_pub_.publish(raw_image_msg)
        self.sync_pcd_pub_.publish(pcd_msg)
    
        self.get_logger().info("Synchronized messages received.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraSynchronizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





