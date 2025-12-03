
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import numpy as np
import cv2
import time

from ids_peak import ids_peak
from ids_peak import ids_peak_ipl_extension

CAM_PARAM_PATH = "/home/wicomai-cv/Documents/sensor_fusion/ros2-drone-localization/ros_ws/src/sensors_driver/config/rgb8.cset"
CAMERA_INDEX = 0
TARGET_FPS = 30

class IDS_CameraPublisher(Node):
    def __init__(self):
        super().__init__('ids_camera_publisher')

        # ROS publishers
        self.pub_image = self.create_publisher(Image, '/ids/image', 10)
        self.pub_info  = self.create_publisher(CameraInfo, '/ids/camera_info', 10)

        # Initialize IDS Peak
        ids_peak.Library.Initialize()
        device_manager = ids_peak.DeviceManager.Instance()
        device_manager.Update()

        if device_manager.Devices().empty():
            self.get_logger().error("No IDS camera found.")
            return

        # Open device
        self.device = device_manager.Devices()[CAMERA_INDEX].OpenDevice(ids_peak.DeviceAccessType_Control)
        self.remote = self.device.RemoteDevice().NodeMaps()[0]

        # Load saved camera params (RGB8)
        self.remote.LoadFromFile(CAM_PARAM_PATH)

        # Force important overrides
        self.remote.FindNode("GainAuto").SetCurrentEntry("Continuous")
        self.remote.FindNode("BalanceWhiteAuto").SetCurrentEntry("Continuous")
        self.remote.FindNode("AcquisitionFrameRate").SetValue(TARGET_FPS)

        # Start streaming
        self.data_stream = self.device.DataStreams()[0].OpenDataStream()
        payload_size = self.remote.FindNode("PayloadSize").Value()
        buffer_count = self.data_stream.NumBuffersAnnouncedMinRequired()

        for _ in range(buffer_count):
            buffer = self.data_stream.AllocAndAnnounceBuffer(payload_size)
            self.data_stream.QueueBuffer(buffer)

        self.remote.FindNode("TLParamsLocked").SetValue(1)
        self.data_stream.StartAcquisition()
        self.remote.FindNode("AcquisitionStart").Execute()

        # Timer to publish frames as fast as possible
        self.timer = self.create_timer(0.0, self.timer_callback)
        self.get_logger().info("IDS Camera Publisher started.")

    def timer_callback(self):
        try:
            buffer = self.data_stream.WaitForFinishedBuffer(50)
            img = ids_peak_ipl_extension.BufferToImage(buffer)

            # RGB IMAGE (NO GRAYSCALE)
            frame = img.get_numpy_3D()      # shape = (H, W, 3)
            height, width, channels = frame.shape

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ids_camera"
            msg.height = height
            msg.width = width
            msg.encoding = "rgb8"           
            msg.step = width * 3          
            msg.data = frame.tobytes()    
            # Publish image
            self.pub_image.publish(msg)

            # Publish dummy CameraInfo
            info = CameraInfo()
            info.header = msg.header
            info.width = width
            info.height = height
            self.pub_info.publish(info)

            # Requeue buffer
            self.data_stream.QueueBuffer(buffer)

        except Exception as e:
            self.get_logger().warn(f"Frame error: {e}")
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IDS_CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
