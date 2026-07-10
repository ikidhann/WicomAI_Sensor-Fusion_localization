import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge


class ImageAndPointCloudExtractor(Node):
    def __init__(self):
        super().__init__('image_and_pointcloud_extractor')

        # --- Topics ---
        image_topic = '/sync/color'
        lidar_topic = '/sync_pcd'

        # --- Subscribers ---
        self.subscription_image = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.subscription_pointcloud = self.create_subscription(
            PointCloud2, lidar_topic, self.pointcloud_callback, 10
        )

        # --- Output directories ---\
        self.image_output_directory = '/home/wicomai-cv/Documents/sensor_fusion/ros2-drone-localization/sync_20m_light/ex_out/color'
        self.pointcloud_output_directory = '/home/wicomai-cv/Documents/sensor_fusion/ros2-drone-localization/sync_20m_light/ex_out/pcd'

        os.makedirs(self.image_output_directory, exist_ok=True)
        os.makedirs(self.pointcloud_output_directory, exist_ok=True)

        self.cv_bridge = CvBridge()
        self.get_logger().info(" Extractor initialized (x, y, z only).")

    # ================== IMAGE CALLBACK ==================
    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp_str = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
            filename = os.path.join(self.image_output_directory, f"image_{timestamp_str}.jpg")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"  Saved image: {filename}")
        except Exception as e:
            self.get_logger().error(f" Image conversion error: {e}")

    # ================== POINT CLOUD CALLBACK ==================
    def pointcloud_callback(self, msg):
        try:
            # Read only x, y, z
            data = np.array(list(point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True
            )))

            if data.size == 0:
                self.get_logger().warn(" Empty point cloud received — skipping.")
                return

            # Convert structured array to plain float32 (N, 3)
            # Handle both structured dtype and normal list output
            if data.dtype.names is not None:
                points = np.vstack([data['x'], data['y'], data['z']]).T.astype(np.float32)
            else:
                points = data.astype(np.float32)

            # Save as .bin
            timestamp_str = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
            filename = os.path.join(self.pointcloud_output_directory, f"pointcloud_{timestamp_str}.bin")
            points.tofile(filename)

            self.get_logger().info(f" Saved point cloud: {filename} ({points.shape[0]} points)")

        except Exception as e:
            self.get_logger().error(f" Point cloud processing error: {e}")


# ================== MAIN ==================
def main(args=None):
    rclpy.init(args=args)
    extractor = ImageAndPointCloudExtractor()
    try:
        rclpy.spin(extractor)
    except KeyboardInterrupt:
        extractor.get_logger().info(" Shutting down extractor node.")
    finally:
        if rclpy.ok():
            extractor.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()