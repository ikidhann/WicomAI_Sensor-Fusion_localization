'''
Code for live streaming ANY image from any camera
====================================================================
'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

# Can be used for regular webcame (Brio, c920, c930) or Intel RealSense L515
cam_type = "Any camera"

# For Intel l515
# 0 is infrared camera
# 6 is RGB camera
cam_source = 0

frame_width = 1920
frame_height = 1080
resize_dim = (frame_width,frame_height)

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/color', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(cam_source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30.0)
        self.bridge = CvBridge()
        self.i = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, resize_dim)
        #frame = cv2.resize(frame, (1920,1080), cv2.INTER_LINEAR)

        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame,'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{self.i}"
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing: frame number %d'%self.i)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()