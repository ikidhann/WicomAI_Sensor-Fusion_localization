import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os

from ids_peak import ids_peak
from ids_peak import ids_peak_ipl_extension

from ament_index_python.packages import get_package_share_directory


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('cam_publisher_node')

        # ------------------------- PARAMETERS -------------------------
        self.declare_parameter('camera_name', 'logitech')
        self.declare_parameter('cam_source', 0)
        self.declare_parameter('frame_width', 1920)
        self.declare_parameter('frame_height', 1080)
        self.declare_parameter('fps', 30.0)

        # Intrinsics
        self.declare_parameter('intrinsics_param.distortion_model', 'plumb_bob')
        self.declare_parameter('intrinsics_param.distortion_coeff', [0.0])
        self.declare_parameter('intrinsics_param.camera_matrix', [0.0])
        self.declare_parameter('intrinsics_param.rectification_matrix', [0.0])
        self.declare_parameter('intrinsics_param.projection_matrix', [0.0])
        

        # ------------------------- GET PARAMS -------------------------
        self.camera_name = self.get_parameter('camera_name').value
        self.fps_ = self.get_parameter('fps').value
        self.cam_source_ = self.get_parameter('cam_source').value
        self.frame_width_ = self.get_parameter('frame_width').value
        self.frame_height_ = self.get_parameter('frame_height').value

        # Build full path to .cset file 
        default_cset_path = 'config/rgb8.cset'
        self.ids_config = os.path.join(
            get_package_share_directory('sensors_driver'),
            default_cset_path
        )
        self.get_logger().info(f"Using IDS CSET file: {self.ids_config}")

        # Intrinsics dict
        self.intrinsics_param_ = {}
        self.intrinsics_param_['distortion_model'] = self.get_parameter('intrinsics_param.distortion_model').value
        self.intrinsics_param_['D'] = self.get_parameter('intrinsics_param.distortion_coeff').value
        self.intrinsics_param_['K'] = self.get_parameter('intrinsics_param.camera_matrix').value
        self.intrinsics_param_['R'] = self.get_parameter('intrinsics_param.rectification_matrix').value
        self.intrinsics_param_['P'] = self.get_parameter('intrinsics_param.projection_matrix').value

        self.get_logger().info(f"Camera Name = {self.camera_name}")
        self.get_logger().info(str(self.intrinsics_param_))

        # ------------------------- PUBLISHERS -------------------------
        self.raw_img_pub_ = self.create_publisher(Image, '/camera/raw_image', 10)
        self.cam_info_pub_ = self.create_publisher(CameraInfo, '/camera/cam_info', 10)

        # ------------------------- CAMERA INITIALIZATION -------------------------
        if self.camera_name == "ids":
            self._init_ids_camera()
        else:
            self._init_opencv_camera()

        # Timer callback
        self.timer = self.create_timer(1.0 / self.fps_, self.timer_callback)
        self.bridge = CvBridge()


    def _init_ids_camera(self):
        self.get_logger().info("Initializing IDS camera...")

        ids_peak.Library.Initialize()
        device_manager = ids_peak.DeviceManager.Instance()
        device_manager.Update()

        if device_manager.Devices().empty():
            self.get_logger().error("No IDS camera detected!")
            return

        self.device = device_manager.Devices()[self.cam_source_].OpenDevice(ids_peak.DeviceAccessType_Control)
        self.remote = self.device.RemoteDevice().NodeMaps()[0]

        try:
            self.remote.LoadFromFile(self.ids_config)
            self.get_logger().info(f"Loaded IDS CSET: {self.ids_config}")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSET file: {e}")

        # Basic settings
        self.remote.FindNode("GainAuto").SetCurrentEntry("Continuous")
        self.remote.FindNode("BalanceWhiteAuto").SetCurrentEntry("Continuous")
        self.remote.FindNode("AcquisitionFrameRate").SetValue(self.fps_)

        # Data stream
        self.data_stream = self.device.DataStreams()[0].OpenDataStream()
        payload_size = self.remote.FindNode("PayloadSize").Value()
        buffer_count = self.data_stream.NumBuffersAnnouncedMinRequired()

        for _ in range(buffer_count):
            buffer = self.data_stream.AllocAndAnnounceBuffer(payload_size)
            self.data_stream.QueueBuffer(buffer)

        self.remote.FindNode("TLParamsLocked").SetValue(1)
        self.data_stream.StartAcquisition()
        self.remote.FindNode("AcquisitionStart").Execute()

        self.get_logger().info("IDS camera initialized successfully.")


    def _init_opencv_camera(self):
        self.get_logger().info("Initializing OpenCV camera...")

        self.cap = cv2.VideoCapture(self.cam_source_)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width_)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height_)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps_)

        self.get_logger().info("OpenCV camera initialized.")


    def _load_camera_info(self):
        cam_info = CameraInfo()
        cam_info.width = self.frame_width_
        cam_info.height = self.frame_height_
        cam_info.k = self.intrinsics_param_['K']
        cam_info.r = self.intrinsics_param_['R']
        cam_info.p = self.intrinsics_param_['P']
        cam_info.d = self.intrinsics_param_['D']
        cam_info.distortion_model = self.intrinsics_param_['distortion_model']
        return cam_info


    def timer_callback(self):

        if self.camera_name == "ids":
            try:
                buffer = self.data_stream.WaitForFinishedBuffer(50)
                img = ids_peak_ipl_extension.BufferToImage(buffer)
                frame = img.get_numpy_3D()
                # self.get_logger().info(f"IMAGE SHAPE: {frame.shape}")
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.data_stream.QueueBuffer(buffer)

            except Exception as e:
                self.get_logger().warn(f"IDS Frame error: {e}", throttle_duration_sec=5.0)
                return

        else:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("No frame from OpenCV camera.", throttle_duration_sec=5.0)
                return

        # ----------------- Publish Image -----------------
        frame = cv2.resize(frame, (self.frame_width_, self.frame_height_))
        time_now = self.get_clock().now().to_msg()
        frame_id = "camera_optical_frame"

        raw_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        raw_msg.header.stamp = time_now
        raw_msg.header.frame_id = frame_id

        cam_info_msg = self._load_camera_info()
        cam_info_msg.header.stamp = time_now
        cam_info_msg.header.frame_id = frame_id

        self.raw_img_pub_.publish(raw_msg)
        self.cam_info_pub_.publish(cam_info_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
