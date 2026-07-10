#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces.msg import NumpyArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np
import os
import cv2

class InstanceSegmentation(Node):
    def __init__(self):
        super().__init__('instance_seg_node')
        
        self.declare_parameter('model_path', 'model/yolov8n-seg.pt')
        self.declare_parameter('confidence_th', 0.4)
        self.declare_parameter('target_class', 0)  # Default 0: person / drone
                            
        model_path = os.path.join(
            get_package_share_directory('sensors_processing'),
            self.get_parameter('model_path').value
        )
 
        self.get_logger().info(f"Segmentation Model: {model_path}")

        self.conf = self.get_parameter('confidence_th').value
        self.target_class = self.get_parameter('target_class').value

        self.cv_bridge_ = CvBridge()
        self.model = YOLO(model_path)
        self.sync_image_sub_ = self.create_subscription(Image, '/sync/raw_image', self.inference_callback, 10)

        self.masks_pub_ = self.create_publisher(NumpyArray, '/drone/masks', 10)
        self.masked_image_pub_ = self.create_publisher(Image, '/drone/masked_image', 10)

        # Exponential moving average for confidence smoothing (alpha=0.3 = more smooth, 0.7 = more reactive)
        self._smooth_conf = 0.0
        self._ema_alpha = 0.3

        self.get_logger().info('InstanceSegmentation node has been started.')
    
    def _yolo_inference(self, cv_image):
        results = self.model.predict(
            cv_image, 
            classes=[self.target_class], 
            conf=self.conf, 
            verbose=False,
            device='cuda:0'
        )
        results = results[0].cpu()
        #print(f"YOLO Inference: {len(results.boxes)} instances detected with confidence > {self.conf}")
        #print(f"YOLO Inference: Detected classes - {results.boxes.cls.numpy()}")
        #print(f"YOLO Inference: Masks - {results.masks.data.numpy() if results.masks is not None else 'None'}")
        
        masked_img, masks = None, None 

        if len(results.boxes.cls) > 0:
            masks = results.masks.data.numpy()  
            img_h, img_w = cv_image.shape[0], cv_image.shape[1]
            masks = np.array([cv2.resize(mask, (img_w, img_h), interpolation=cv2.INTER_NEAREST) for mask in masks])
            masked_img = results.plot(boxes=False, color_mode='instance')

            # Get highest confidence detection and apply EMA smoothing
            conf_values = results.boxes.conf.numpy()
            raw_conf = float(conf_values.max())
            self._smooth_conf = self._ema_alpha * raw_conf + (1 - self._ema_alpha) * self._smooth_conf

            cv2.putText(masked_img, f'Conf: {raw_conf:.2f} ', (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            #self.get_logger().info(f"Detection: {raw_conf:.2f}")
            

        #self.get_logger().info(f"Masks count: {len(masks) if masks is not None else 0}")
        return masked_img, masks
    
    def _nparray_to_msg(self, arr):
        msg = NumpyArray()
        msg.data = arr.astype(np.float32).flatten().tolist()
        msg.shape = [int(x) for x in arr.shape]

        return msg

    def inference_callback(self, raw_image_msg):
        cv_image = self.cv_bridge_.imgmsg_to_cv2(raw_image_msg, 'bgr8')

        try:
            
            masked_img, masks = self._yolo_inference(cv_image)

            if masks is None:
                #self.get_logger().info('No Drone Instances Detected!')
                return

        except Exception as e:
            self.get_logger().warn(f'Inference error in InstanceSegmentation node: {e}')
            return
        
        masked_img_msg = self.cv_bridge_.cv2_to_imgmsg(masked_img, 'bgr8')
        masked_img_msg.header = raw_image_msg.header
        seg_masks_msg = self._nparray_to_msg(masks)
        seg_masks_msg.header = raw_image_msg.header
        
        try:
            self.masks_pub_.publish(seg_masks_msg)
            self.masked_image_pub_.publish(masked_img_msg)
        except Exception as e:
            self.get_logger().error(f'Publishing error in InstanceSegmentation node: {e}')

def main(args=None):
    rclpy.init(args=args)

    try:
        node = InstanceSegmentation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':  
    main()