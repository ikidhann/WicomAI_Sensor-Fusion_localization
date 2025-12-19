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
        

    def _lidar_projection(self, transformed_pcd, P, img_h, img_w):

        pc_3d = pc2.read_points_numpy(transformed_pcd, field_names=("x", "y", "z"), skip_nans=True).T
        N_points = pc_3d.shape[1]

        pc_3d_hom = np.vstack((pc_3d, np.ones((1, N_points))))
        pc_2d_hom = P @ pc_3d_hom

        # Filter points in front of cam
        z_cam = pc_2d_hom[2, :]
        filter = z_cam > 0.1

        pc_2d_hom = pc_2d_hom[:, filter]
        z_cam = z_cam[filter]
        N_in_front = pc_2d_hom.shape[1]

        # Extract pixel coordinates
        u = (pc_2d_hom[0, :] / z_cam).astype(np.int32)
        v = (pc_2d_hom[1, :] / z_cam).astype(np.int32)

        # Filter points within frame
        filter = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
        projected_pcd = np.vstack([u, v, z_cam]).T
        projected_pcd = projected_pcd[filter, :]
        N_in_frame = projected_pcd.shape[0]

        log_msg = f'Points: Total={N_points} | InFront={N_in_front} | InFrame={N_in_frame}'

        self.get_logger().info(
            log_msg,
            throttle_duration_sec=1.0
        )

        return projected_pcd

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

        projected_pcd = self._lidar_projection(transformed_pcd, P, IMG_H, IMG_W)

        for point in projected_pcd:
            u, v, z_cam = point
            u, v = int(u), int(v)
            
            depth_color = min(255, int(z_cam * 20)) 
            cv2.circle(cv_image, (u, v), radius=2, color=(0, 255-depth_color, depth_color), thickness=-1)

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
    
    try:
        node = LidarCameraProjection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':  
    main()