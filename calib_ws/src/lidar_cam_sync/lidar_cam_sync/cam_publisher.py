'''
Code for live streaming ANY image from any camera
====================================================================
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

# Can be used for regular webcame (Brio, c920, c930) or Intel RealSense L515
# For Intel l515
# 0 is infrared camera
# 6 is RGB camera

# TODO: change the camera setup for IDS camera
# TODO: find intrinsics params for IDS camera

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('came_publisher_node')
        self.declare_parameter('cam_source', 0)
        self.declare_parameter('frame_width', 1920)
        self.declare_parameter('frame_height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('intrinsics_params.distortion_model', 'plumb_bob')
        self.declare_parameter('intrinsics_params.distortion_coeff', [0])
        self.declare_parameter('intrinsics_params.camera_matrix', [0])
        self.declare_parameter('intrinsics_params.rectification_matrix', [0])
        self.declare_parameter('intrinsics_params.projection_matrix', [0])        
        
        # Get all params
        self.cam_source_ = self.get_parameter('cam_source').get_parameter_value().integer_value
        self.frame_width_ = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height_ = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.fps_ = self.get_parameter('fps').get_parameter_value().double_value
        self.intrinsics_params_ = {}
        self.intrinsics_params_['distortion_model'] = self.get_parameter('intrinsics_params.distortion_model').get_parameter_value().string_value
        self.intrinsics_params_['D'] = self.get_parameter('intrinsics_params.distortion_coeff').get_parameter_value().double_array_value
        self.intrinsics_params_['K'] = self.get_parameter('intrinsics_params.camera_matrix').get_parameter_value().double_array_value
        self.intrinsics_params_['R'] = self.get_parameter('intrinsics_params.rectification_matrix').get_parameter_value().double_array_value
        self.intrinsics_params_['P'] = self.get_parameter('intrinsics_params.projection_matrix').get_parameter_value().double_array_value


        self.raw_img_pub_ = self.create_publisher(Image, '/camera/raw_image', 10)
        self.cam_info_pub_ = self.create_publisher(CameraInfo, '/camera/cam_info', 10)

        timer_period = 1.0 / self.fps_  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.cap = cv2.VideoCapture(self.cam_source_)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width_)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height_)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps_)
        
        self.bridge = CvBridge()
        self.i = 0
    
    def _load_camera_info(self):
        cam_info = CameraInfo()
        cam_info.width = self.frame_width_
        cam_info.height = self.frame_height_
        cam_info.k = self.intrinsics_params_['K']
        cam_info.r = self.intrinsics_params_['R']
        cam_info.p = self.intrinsics_params_['P']
        cam_info.d = self.intrinsics_params_['D']
        cam_info.distortion_model = self.intrinsics_params_['distortion_model']
        
        return cam_info
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        resize_dim = (self.frame_width_, self.frame_height_)
        frame = cv2.resize(frame, resize_dim)

        if ret:
            msg_id = f"{self.i}"
            
            raw_image_msg = self.bridge.cv2_to_imgmsg(frame,'bgr8')
            raw_image_msg.header.stamp = self.get_clock().now().to_msg()
            raw_image_msg.header.frame_id = msg_id

            cam_info_msg = self._load_camera_info()
            cam_info_msg.header.stamp = self.get_clock().now().to_msg()
            cam_info_msg.header.frame_id = msg_id

            # Publish messages
            self.raw_img_pub_.publish(raw_image_msg)
            self.cam_info_pub_.publish(cam_info_msg)

        else:
            self.get_logger().info(f'[ID={self.i}] No frame captured from camera')

        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    cam_publisher = CameraPublisher()
    rclpy.spin(cam_publisher)
    cam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()