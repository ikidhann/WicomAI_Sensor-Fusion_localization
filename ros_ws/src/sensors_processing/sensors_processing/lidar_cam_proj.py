import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import tf2_ros
import tf2_sensor_msgs
import numpy as np
import cv2

class LidarCameraProjection(Node):
    def __init__(self):
        super().__init__('lidar_camera_proj_node')

        self.source_frame_ = 'velodyne'
        self.target_frame_ = 'camera_optical_frame'
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.sync_pcd_sub_ = message_filters.Subscriber(self, PointCloud2, '/sync/velodyne_points')
        self.sync_image_sub_ = message_filters.Subscriber(self, Image, '/sync/raw_image')
        self.sync_cam_info_sub_ = message_filters.Subscriber(self, CameraInfo, '/sync/cam_info')

        self.ts_ = message_filters.ApproximateTimeSynchronizer(
            [self.sync_pcd_sub_, self.sync_image_sub_, self.sync_cam_info_sub_],
            queue_size=5,
            slop=0.05 # seconds
        )
        
        self.ts_.registerCallback(self.projection_callback)
        self.projection_pub_ = self.create_publisher(Image, '/lidar_cam_proj', 10)
        self.bridge_ = CvBridge()
        self.get_logger().info("Lidar to Camera Projection Node started!")
        

    def _lidar_projection(self, transformed_pcd, cv_image, P, img_h, img_w):
        point_count = 0
        points_in_front = 0
        points_in_frame = 0

        for point in pc2.read_points(transformed_pcd, field_names=("x", "y", "z"), skip_nans=True):
            point_count += 1
            x_cam, y_cam, z_cam = point
            
            # Filter out points behind the camera
            if z_cam <= 0.1: # meter
                continue
            points_in_front += 1
            
            # Project 3D point (x_cam, y_cam, z_cam) to 2D pixel (u, v)
            # Create a 4x1 homogeneous point
            point_3d_hom = np.array([x_cam, y_cam, z_cam, 1.0])
            point_2d_hom = P @ point_3d_hom
            
            # Normalize to get pixel coordinates
            u = int(point_2d_hom[0] / point_2d_hom[2])
            v = int(point_2d_hom[1] / point_2d_hom[2])

            # Visualize the point on the image
            if 0 <= u < img_w and 0 <= v < img_h:
                points_in_frame += 1
                # Color based on depth (z_cam)
                depth_color = min(255, int(z_cam * 20)) 
                cv2.circle(cv_image, (u, v), radius=2, color=(0, 255-depth_color, depth_color), thickness=-1)
            
        self.get_logger().info(
            f'Points: Total={point_count} | InFront={points_in_front} | InFrame={points_in_frame}',
            throttle_duration_sec=1.0
        )

    def projection_callback(self, pcd_msg, raw_image_msg, cam_info_msg):
        try:
            tf = self.tf_buffer_.lookup_transform(
                self.target_frame_,   
                self.source_frame_, 
                rclpy.time.Time() 
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.source_frame_} to {self.target_frame_}: {ex}',
                throttle_duration_sec=1.0
            )
            return

        transformed_pcd = tf2_sensor_msgs.do_transform_cloud(pcd_msg, tf)
        cv_image = self.bridge_.imgmsg_to_cv2(raw_image_msg, 'bgr8')
        IMG_H, IMG_W, _ = cv_image.shape

        # get camera projection matrix (P) from CameraInfo
        P = np.array(cam_info_msg.p).reshape(3, 4)

        self._lidar_projection(transformed_pcd, cv_image, P, IMG_H, IMG_W)

        # publish the projected image
        try:
            projection_image_msg = self.bridge_.cv2_to_imgmsg(cv_image, 'bgr8')
            projection_image_msg.header = raw_image_msg.header 
            self.projection_pub_.publish(projection_image_msg)
            # self.get_logger().info("Projected pcd to image published.")
        except Exception as e:
            self.get_logger().error(f'Failed to publish projection image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':  
    main()