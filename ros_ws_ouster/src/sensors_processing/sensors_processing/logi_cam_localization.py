import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
from ultralytics import YOLO
import yaml
import numpy as np

# defining the fonts
fonts = cv2.FONT_HERSHEY_COMPLEX

# width of drpne in the real world or Object Plane
# centimeter
Known_width = 90.0

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

class CameraDistEstim(Node):
    def __init__(self):
        super().__init__('lidar_cam_sync_node')

        self.declare_parameter('config_path', 'config/config.yaml')
        self.declare_parameter('model_path', 'model/drone-seg_best.pt')
        self.declare_parameter('confidence_th', 0.6)
        self.declare_parameter('target_class', 0)  # Default 0: person / drone

        model_path = os.path.join(
            get_package_share_directory('sensors_processing'),
            self.get_parameter('model_path').value
        )

        self.config_paths = os.path.join(
            get_package_share_directory('sensors_bringup'),
            self.get_parameter('config_path').value
        )
 
        self.get_logger().info(f"Segmentation Model: {model_path}")

        self.conf = self.get_parameter('confidence_th').value
        self.target_class = self.get_parameter('target_class').value

        self.cv_bridge_ = CvBridge()
        self.model = YOLO(model_path)
        self.sync_image_sub_ = self.create_subscription(Image, '/sync/raw_image', self.inference_callback, 10)
        
        # Publishers for viewing synchronized messages
        self.results_image = self.create_publisher(Image, '/drone/cam_est_distance', 10)

        self.get_logger().info('Camera Only Localization node has been started.')
        self.get_logger().info("CameraEstim Node started!")

    def _load_focal_lengths(self, path):
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)

        # Navigate the hierarchy: cam_publisher_node -> ros__parameters -> intrinsics_param
        intr = cfg["cam_publisher_node"]["ros__parameters"]["intrinsics_param"]

        # camera_matrix is a flat list of 9 elements [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        K = np.array(intr["camera_matrix"], dtype=float).reshape(3, 3)

        fx = K[0, 0]
        fy = K[1, 1]
        return fx, fy

    # distance estimation function
    def _distance_finder(self, Focal_Length, real_drone_width, drone_width_in_frame):
        distance = (real_drone_width * Focal_Length)/drone_width_in_frame
        return distance
    
    def _draw_distance_info(self, cv_image, distance):
        distance = distance / 100.0
        text = f"Dist. to Target: {distance:.3f} m"
        pos = (10, 60)
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 2
        color = (128, 239, 128) #EF80EF
        thickness = 2

        (w, h), _ = cv2.getTextSize(text, font, font_scale, thickness)
        cv2.rectangle(cv_image, (pos[0], pos[1] - h - 10), (pos[0] + w, pos[1] + 10), (0, 0, 0), -1)
        cv2.putText(cv_image, text, pos, font, font_scale, color, thickness)

    def _yolo_inference(self, cv_image):
        results = self.model.predict(
            cv_image, 
            classes=[self.target_class], 
            conf=self.conf, 
            verbose=False,
            device='cuda:0'
        )

        boxes = results[0].boxes
        xywh = boxes.xywh.cpu()

        output_image = None
        w = 0.0
        h = 0.0

        for box in xywh:
            cx, cy, w, h = box.tolist()

            # convert the coordinates
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            # draw bbox
            output_image = cv2.rectangle(cv_image,
                                         (x1, y1),
                                         (x2, y2),
                                         RED, 2)
            
        return output_image, w, h
    
    def inference_callback(self, image_msg):
        cv_image = self.cv_bridge_.imgmsg_to_cv2(image_msg, 'bgr8')
        
        """ Detecting bounding boxes """
        try:
            bboxes, pixel_width, pixel_height = self._yolo_inference(cv_image)

            if bboxes is None:
                self.get_logger().info('No Drone Instances Detected!')
                return
            
        except Exception as e:
            self.get_logger().warn(f'Inference error in Bbox node: {e}')
            return

        fx, fy = self._load_focal_lengths(self.config_paths)

        Distance = self._distance_finder(fx,
                                         Known_width,
                                         pixel_width)
            
        self._draw_distance_info(bboxes, Distance)

        # Publish localized drone in the frame
        try:
            localized_img = self.cv_bridge_.cv2_to_imgmsg(bboxes, 'bgr8')
            localized_img.header = image_msg.header 
            self.results_image.publish(localized_img)
        except Exception as e:
            self.get_logger().error(f'Failed to publish projection image: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraDistEstim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()