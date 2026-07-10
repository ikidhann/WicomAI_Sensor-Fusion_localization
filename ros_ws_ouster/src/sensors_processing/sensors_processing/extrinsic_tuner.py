import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np
import scipy.spatial.transform as T

class ExtrinsicTuner(Node):
    def __init__(self):
        super().__init__('extrinsic_tuner_node')

        # Load initial extrinsic params
        T_lidar_cam = [
            0.020277607258978564, 0.03658924142983973, -0.0945841065101958,
            -0.48278176460463185, -0.5093193574706256, 0.515196897292822, -0.4920241019409478
        ]

        self.T_base = np.eye(4)
        self.T_base[:3, 3] = T_lidar_cam[:3]  # Translation
        self.T_base[:3, :3] = T.Rotation.from_quat(T_lidar_cam[3:]).as_matrix()  # Rotation

        self.window_name = "Extrinsic Calibration Tuner"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.limit_T = 2000 # +/- 2 m
        self.scale_T = 1000.0 
        self.limit_R = 1800 # +/- 180 degrees  
        self.scale_R = 10.0

        self.latest = None
        self.current = None
        self.is_update_frame = True

        self._create_trackbar('Tx', self.limit_T)
        self._create_trackbar('Ty', self.limit_T)
        self._create_trackbar('Tz', self.limit_T)
        self._create_trackbar('Roll', self.limit_R)
        self._create_trackbar('Pitch', self.limit_R)
        self._create_trackbar('Yaw', self.limit_R)

        self.sync_pcd_sub_ = message_filters.Subscriber(self, PointCloud2, '/sync/ouster_points')
        self.sync_image_sub_ = message_filters.Subscriber(self, Image, '/sync/raw_image')
        self.sync_cam_info_sub_ = message_filters.Subscriber(self, CameraInfo, '/sync/cam_info')

        self.ts_ = message_filters.ApproximateTimeSynchronizer(
            [self.sync_pcd_sub_, self.sync_image_sub_, self.sync_cam_info_sub_],
            #queue_size=4, slop=0.05
            #add
            queue_size=30, slop=0.05
        )
        self.ts_.registerCallback(self.callback)
        self.bridge_ = CvBridge()

        self.create_timer(0.05, self.gui)
        self.get_logger().info("Extrinsic Tuner Node started!")
    
    
    def _get_current_transform(self):
        dx = (cv2.getTrackbarPos('Tx', self.window_name) - self.limit_T) / self.scale_T
        dy = (cv2.getTrackbarPos('Ty', self.window_name) - self.limit_T) / self.scale_T
        dz = (cv2.getTrackbarPos('Tz', self.window_name) - self.limit_T) / self.scale_T
        
        d_roll = (cv2.getTrackbarPos('Roll', self.window_name) - self.limit_R) / self.scale_R
        d_pitch = (cv2.getTrackbarPos('Pitch', self.window_name) - self.limit_R) / self.scale_R
        d_yaw = (cv2.getTrackbarPos('Yaw', self.window_name) - self.limit_R) / self.scale_R

        T_offset = np.eye(4)
        T_offset[:3, :3] = T.Rotation.from_euler('xyz', [d_roll, d_pitch, d_yaw], degrees=True).as_matrix()
        T_offset[:3, 3] = [dx, dy, dz]

        T_final = T_offset @ self.T_base 
        
        return T_final


    def _create_trackbar(self, name, limit):
        cv2.createTrackbar(name, self.window_name, limit, limit * 2, lambda x: None)


    def _print_result(self, T_lidar_cam):
        trans = T_lidar_cam[:3, 3]
        rot = T.Rotation.from_matrix(T_lidar_cam[:3, :3])
        quat = rot.as_quat()

        self.get_logger().info("FINAL CALIBRATION RESULT (Base + Offset Applied)")
        self.get_logger().info(f"""[
            # Translation
            {trans[0]}, {trans[1]}, {trans[2]}, 
            # Rotation (Quaternion)
            {quat[0]}, {quat[1]}, {quat[2]}, {quat[3]}
        ]""")


    def _pc_projection(self, pc_3d, T_lidar_cam, P, img_h, img_w):

        N_points = pc_3d.shape[1]

        pc_3d_hom = np.vstack((pc_3d, np.ones((1, N_points))))
        pc_3d_hom = T_lidar_cam @ pc_3d_hom # project to cam 3D frame
        pc_2d_hom = P @ pc_3d_hom # 3D to 2D

        # Filter points in front of cam
        z_cam = pc_2d_hom[2, :]
        filter = z_cam > 0.1

        pc_2d_hom = pc_2d_hom[:, filter]
        z_cam = z_cam[filter]
        #add (original commnet off)
        N_in_front = pc_2d_hom.shape[1]

        # Extract pixel coordinates
        u = (pc_2d_hom[0, :] / z_cam).astype(np.int32)
        v = (pc_2d_hom[1, :] / z_cam).astype(np.int32)

        # Filter points within frame
        filter = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
        projected_pcd = np.vstack([u, v, z_cam]).T
        projected_pcd = projected_pcd[filter, :]
        #add (original commnet off)
        N_in_frame = projected_pcd.shape[0]

        
        # log_msg = f'Points: Total={N_points} | InFront={N_in_front} | InFrame={N_in_frame}'
        #Add
        self.get_logger().info(
            f'[DEBUG] Total={N_points} | InFront={N_in_front} | InFrame={N_in_frame} | '
            f'u=[{u.min()},{u.max()}] v=[{v.min()},{v.max()}] img=[{img_w}x{img_h}]',
            throttle_duration_sec=2.0
        )
        return projected_pcd
    
    def callback(self, pcd_msg, raw_image_msg, cam_info_msg):
        self.latest = [pcd_msg, raw_image_msg, cam_info_msg]


    def gui(self):
        if self.is_update_frame and self.latest:
            pcd_msg, raw_image_msg, cam_info_msg = self.latest
            cv_image = self.bridge_.imgmsg_to_cv2(raw_image_msg, 'bgr8')
            pcd = pc2.read_points_numpy(pcd_msg, field_names=("x", "y", "z"), skip_nans=True).T
            P = np.array(cam_info_msg.p).reshape(3, 4)

            self.current = dict(
                image=cv_image,
                pcd=pcd,
                P=P
            )
            self.is_update_frame = False
            self.get_logger().info("Frame updated...")

        
        if self.current:
            display_img = self.current['image'].copy()
            H, W, _= display_img.shape
            
            T_final = self._get_current_transform()
            
            projected_pcd = self._pc_projection(self.current['pcd'], T_final, self.current['P'], H, W)

            for point in projected_pcd:
                u, v, z = point
                val = min(255, int(z * 20))
                cv2.circle(display_img, (int(u), int(v)), 2, (0, 255 - val, val), -1)

            cv2.putText(display_img, "[N]Frame [S]Save [I/K]Pitch [J/L]Yaw [U/O]Roll [Q/E]Ty [A/D]Tx [Z/X]Tz", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow(self.window_name, display_img)
        
        key = cv2.waitKey(1)
        step_T = 10   # 10mm per keypress
        step_R = 10   # 1 degree per keypress

        if key == ord('n') or key == ord('N'):
            self.get_logger().info("Next Frame...")
            self.is_update_frame = True
        elif key == ord('s') or key == ord('S'):
            T_final = self._get_current_transform()
            self._print_result(T_final)
        elif key == 27:
            raise KeyboardInterrupt
        # Translation keys
        elif key == ord('d'): cv2.setTrackbarPos('Tx', self.window_name, cv2.getTrackbarPos('Tx', self.window_name) + step_T)
        elif key == ord('a'): cv2.setTrackbarPos('Tx', self.window_name, cv2.getTrackbarPos('Tx', self.window_name) - step_T)
        elif key == ord('e'): cv2.setTrackbarPos('Ty', self.window_name, cv2.getTrackbarPos('Ty', self.window_name) + step_T)
        elif key == ord('q'): cv2.setTrackbarPos('Ty', self.window_name, cv2.getTrackbarPos('Ty', self.window_name) - step_T)
        elif key == ord('z'): cv2.setTrackbarPos('Tz', self.window_name, cv2.getTrackbarPos('Tz', self.window_name) + step_T)
        elif key == ord('x'): cv2.setTrackbarPos('Tz', self.window_name, cv2.getTrackbarPos('Tz', self.window_name) - step_T)
        # Rotation keys
        elif key == ord('i'): cv2.setTrackbarPos('Pitch', self.window_name, cv2.getTrackbarPos('Pitch', self.window_name) + step_R)
        elif key == ord('k'): cv2.setTrackbarPos('Pitch', self.window_name, cv2.getTrackbarPos('Pitch', self.window_name) - step_R)
        elif key == ord('j'): cv2.setTrackbarPos('Yaw',   self.window_name, cv2.getTrackbarPos('Yaw',   self.window_name) - step_R)
        elif key == ord('l'): cv2.setTrackbarPos('Yaw',   self.window_name, cv2.getTrackbarPos('Yaw',   self.window_name) + step_R)
        elif key == ord('u'): cv2.setTrackbarPos('Roll',  self.window_name, cv2.getTrackbarPos('Roll',  self.window_name) - step_R)
        elif key == ord('o'): cv2.setTrackbarPos('Roll',  self.window_name, cv2.getTrackbarPos('Roll',  self.window_name) + step_R)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExtrinsicTuner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()