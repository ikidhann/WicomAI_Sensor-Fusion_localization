import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from custom_interfaces.msg import NumpyArray
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import tf2_ros
import tf2_sensor_msgs
import numpy as np
import cv2

class DroneLocalization(Node):
    def __init__(self):
        super().__init__('drone_localization_node')

        self.source_frame_ = 'velodyne'
        self.target_frame_ = 'camera_optical_frame'
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.sync_pcd_sub_ = message_filters.Subscriber(self, PointCloud2, '/sync/velodyne_points')
        self.sync_image_sub_ = message_filters.Subscriber(self, Image, '/sync/raw_image')
        self.sync_cam_info_sub_ = message_filters.Subscriber(self, CameraInfo, '/sync/cam_info')
        self.sync_mask_sub_ = message_filters.Subscriber(self, NumpyArray, '/drone/masks')


        self.ts_ = message_filters.ApproximateTimeSynchronizer(
            [self.sync_pcd_sub_, self.sync_image_sub_, self.sync_cam_info_sub_, self.sync_mask_sub_],
            queue_size=5,
            slop=0.05 # seconds
        )
        self.ts_.registerCallback(self.localization_callback)
        
        self.localized_pcd_ = self.create_publisher(Image, '/drone/localized_pcd', 10)
        # self.distance_pub_ = self.create_publisher(Float64MultiArray, '/drone/distances', 10)
        self.bridge_ = CvBridge()
        self.get_logger().info("Drone Localization Node started!")
        

    def _pc_projection(self, transformed_pcd, P, img_h, img_w):

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

        return projected_pcd, log_msg

    
    def _pc_localization(self, projected_pcd, cv_image, masks, threshold=0.1):
        try:
            masks = masks[0]
            distances = []

            for point in projected_pcd:
                u, v, z_cam = point
                u, v = int(u), int(v)
            
                if masks[v, u] > threshold:
                    distances.append(z_cam)
                    depth_color = min(255, int(z_cam * 20)) 
                    cv2.circle(cv_image, (u, v), radius=2, color=(0, 255-depth_color, depth_color), thickness=-1)

            if len(distances) != 0:
                relative_distance = np.array(distances).min()
                #relative_distance = np.quantile(np.array(distances), 0.01)
             
                # upper_bound = int(len(distances) * 0.05)
                # distances = np.array(sorted(distances))
                # relative_distance = np.mean(distances[:upper_bound])

                log_msg = f'Localized Drone Distance: {relative_distance:.2f} meters'
                # self.get_logger().info(f'Drone Distances: {len(distances)}', throttle_duration_sec=1.0)
            else:
                log_msg = 'Localized Drone Distance: None'

            self.get_logger().info(
                log_msg,
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().warn(
                f'Localization failed: {e}',
                throttle_duration_sec=1.0
            )

    def localization_callback(self, pcd_msg, raw_image_msg, cam_info_msg, drone_mask_msg):
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

        transformed_pcd = tf2_sensor_msgs.do_transform_cloud(pcd_msg, tf)
        drone_masks = np.array(drone_mask_msg.data, dtype=np.float32).reshape(drone_mask_msg.shape)
        cv_image = self.bridge_.imgmsg_to_cv2(raw_image_msg, 'bgr8')

        # get camera projection matrix (P) from CameraInfo
        P = np.array(cam_info_msg.p).reshape(3, 4)
        IMG_H, IMG_W, _ = cv_image.shape

        # get projected pcd and localize drone
        projected_pcd, proj_info = self._pc_projection(transformed_pcd, P, IMG_H, IMG_W)
        self.get_logger().info(proj_info, throttle_duration_sec=1.0)

        self._pc_localization(projected_pcd, cv_image, drone_masks)


        # Publish localized drone in the frame
        try:
            localized_drone_msg = self.bridge_.cv2_to_imgmsg(cv_image, 'bgr8')
            localized_drone_msg.header = raw_image_msg.header 
            self.localized_pcd_.publish(localized_drone_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish projection image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DroneLocalization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':  
    main()