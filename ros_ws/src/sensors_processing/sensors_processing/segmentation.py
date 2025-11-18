#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np
import os


class InstanceSegmentation(Node):
    def __init__(self):
        super().__init__('instance_seg_node')
        
        default_model_path = 'model/yolov8n-seg.pt'
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('confidence_th', 0.6)
        self.declare_parameter('target_class', 0)  # Default 0: person / drone

        model_path = os.path.join(
            get_package_share_directory('sensors_processing'),
            self.get_parameter('model_path').value
        )

        self.conf = self.get_parameter('confidence_th').value
        self.target_class = self.get_parameter('target_class').value

        self.cv_bridge_ = CvBridge()
        self.model = YOLO(model_path)
        self.sync_image_sub_ = self.create_subscription(Image, '/sync/raw_image', self.inference_callback, 10)

        self.masks_pub_ = self.create_publisher(Float32MultiArray, '/drone/masks', 10)
        self.masked_image_pub_ = self.create_publisher(Image, '/drone/masked_image', 10)

        self.get_logger().info('InstanceSegmentation node has been started.')
    
    def _yolo_inference(self, cv_image):
        results = self.model.predict(
            cv_image, 
            classes=[self.target_class], 
            conf=self.conf, 
            verbose=False
        )

        results = results[0].cpu()
        masks = results.masks.data.numpy()
        masks = masks.astype(np.float32)
        masked_img = results.plot(boxes=False, color_mode='instance')

        return masked_img, masks
    
    def _nparray_to_msg(self, arr):
        msg = Float32MultiArray()
        msg.data = arr.flatten().tolist()
        
        for i, size in enumerate(arr.shape):
            dim = MultiArrayDimension()
            dim.size = size
            dim.stride = int(np.prod(arr.shape[i+1:])) if i < len(arr.shape) - 1 else 1
            msg.layout.dim.append(dim)

        return msg

    def inference_callback(self, raw_image_msg):
        cv_image = self.cv_bridge_.imgmsg_to_cv2(raw_image_msg, 'bgr8')
        masked_img, masks = self._yolo_inference(cv_image)

        masked_img_msg = self.cv_bridge_.cv2_to_imgmsg(masked_img, 'bgr8')
        masked_img_msg.header = raw_image_msg.header
        seg_masks_msg = self._nparray_to_msg(masks)
        # seg_masks_msg.header = raw_image_msg.header
        
        self.masks_pub_.publish(seg_masks_msg)
        self.masked_image_pub_.publish(masked_img_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = InstanceSegmentation()
        rclpy.spin(node)
    except Exception as e:
        print(f'Exception in InstanceSegmentation node: {e}')
        raise(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  
    main()