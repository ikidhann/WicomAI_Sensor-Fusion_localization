import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation

class StaticTfBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        
        self.declare_parameter('T_lidar_camera', [0.0]*7)  # [x, y, z, qx, qy, qz, qw]
        T_lidar_camera = self.get_parameter('T_lidar_camera').value

        # T_lidar_camera values represent the lidar->camera transform (as used in the tuner:
        # p_cam = T @ p_lidar). However, ROS2 TF2 with parent='os_lidar', child='camera_optical_frame'
        # stores a transform that TF2 interprets as camera->lidar (child->parent direction).
        # lookup_transform('camera_optical_frame', 'os_lidar') then returns the INVERSE.
        # To make the projection node receive the correct lidar->camera transform, we must
        # publish the inverse here so TF2's inversion cancels it out.
        T_mat = np.eye(4)
        T_mat[:3, 3] = T_lidar_camera[:3]
        T_mat[:3, :3] = Rotation.from_quat(T_lidar_camera[3:]).as_matrix()
        T_inv = np.linalg.inv(T_mat)
        t_inv = T_inv[:3, 3]
        q_inv = Rotation.from_matrix(T_inv[:3, :3]).as_quat()  # [qx, qy, qz, qw]

        self.tf_broadcaster_ = tf2_ros.StaticTransformBroadcaster(self)
        tf = TransformStamped()
        
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'os_lidar'
        tf.child_frame_id = 'camera_optical_frame'
        
        # Translation (inverted)
        tf.transform.translation.x = t_inv[0]
        tf.transform.translation.y = t_inv[1]
        tf.transform.translation.z = t_inv[2]
        
        # Rotation (Quaternion, inverted)
        tf.transform.rotation.x = q_inv[0]
        tf.transform.rotation.y = q_inv[1]
        tf.transform.rotation.z = q_inv[2]
        tf.transform.rotation.w = q_inv[3]

        self.tf_broadcaster_.sendTransform(tf)
        self.get_logger().info("Published static transform from 'os_lidar' to 'camera_optical_frame'.")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StaticTfBroadcaster()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()