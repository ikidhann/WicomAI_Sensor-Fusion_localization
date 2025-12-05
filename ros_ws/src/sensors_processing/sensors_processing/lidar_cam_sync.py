import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import message_filters

class LidarCameraSync(Node):
    def __init__(self):
        super().__init__('lidar_cam_sync_node')
        self.pcd_sub_ = message_filters.Subscriber(self, PointCloud2,'/velodyne_points')
        self.raw_img_sub_ = message_filters.Subscriber(self, Image, '/camera/raw_image')
        self.cam_info_sub_ = message_filters.Subscriber(self, CameraInfo, '/camera/cam_info')

        # Publishers for viewing synchronized messages
        self.sync_pcd_pub_ = self.create_publisher(PointCloud2, "/sync/velodyne_points", 10)
        self.sync_raw_img_pub_ = self.create_publisher(Image, "/sync/raw_image", 10)
        self.sync_cam_info_pub_ = self.create_publisher(CameraInfo, "/sync/cam_info", 10)

        self.ts_ = message_filters.ApproximateTimeSynchronizer(
            [self.pcd_sub_, self.raw_img_sub_, self.cam_info_sub_],
            queue_size=5,
            slop=0.05, # seconds
            allow_headerless=False
        )

        self.ts_.registerCallback(self.synchronized_callback)
        self.get_logger().info("LidarCameraSync Node started!")
    
    
    def synchronized_callback(self, pcd_msg:PointCloud2, raw_image_msg:Image, cam_info_msg:CameraInfo):
        pcd_time = Time.from_msg(pcd_msg.header.stamp)
        img_time = Time.from_msg(raw_image_msg.header.stamp)

        pcd_img_dt = abs((pcd_time - img_time).nanoseconds)

        # Check synchornization deltas
        self.get_logger().info(
            f'[PCD-IMG] Sync time deltas: {pcd_img_dt / 1e6:.2f} ms',
            throttle_duration_sec=1.0
        )

        sync_time = raw_image_msg.header.stamp
        pcd_msg.header.stamp = sync_time
        raw_image_msg.header.stamp = sync_time
        cam_info_msg.header.stamp = sync_time

        self.sync_pcd_pub_.publish(pcd_msg)
        self.sync_raw_img_pub_.publish(raw_image_msg)
        self.sync_cam_info_pub_.publish(cam_info_msg)
    
        # self.get_logger().info("Synchronized messages published")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LidarCameraSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()





