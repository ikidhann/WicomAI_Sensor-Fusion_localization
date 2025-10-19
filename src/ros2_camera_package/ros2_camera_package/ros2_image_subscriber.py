#!/usr/bin/env python3
"""
 Subscriber node for images published on /camera/color.
 
 Usage: run this node while the publisher is running. It will display
 incoming frames using OpenCV and log frame ids.
 """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
 
 
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # Subscribe to the same topic as the publisher
        self.subscription = self.create_subscription(
            Image,
            '/camera/color',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.bridge = CvBridge()
        self.frame_count = 0
 
    def listener_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
 
        # Optional: show the image in a window
        window_name = 'regularcam_sub - /camera/color'
        cv2.imshow(window_name, cv_image)
        # Use waitKey(1) for non-blocking display; necessary to refresh window
        cv2.waitKey(1)
 
        # Log some limited information
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Received frame #{self.frame_count} header.frame_id={msg.header.frame_id}')
 
    def destroy_node(self):
        # Clean up OpenCV windows on shutdown
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()
 
 
def main(args=None):
        rclpy.init(args=args)
        node = ImageSubscriber()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()