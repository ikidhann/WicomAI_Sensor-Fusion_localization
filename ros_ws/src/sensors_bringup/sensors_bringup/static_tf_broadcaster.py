import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticTfBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        
        self.declare_parameter('T_lidar_camera', [0.0]*7)  # [x, y, z, qx, qy, qz, qw]
        T_lidar_camera = self.get_parameter('T_lidar_camera').value

        self.tf_broadcaster_ = tf2_ros.StaticTransformBroadcaster(self)
        tf = TransformStamped()
        
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'velodyne'
        tf.child_frame_id = 'camera_optical_frame'
        
        # Translation
        tf.transform.translation.x = T_lidar_camera[0]
        tf.transform.translation.y = T_lidar_camera[1]
        tf.transform.translation.z = T_lidar_camera[2]
        
        # Rotation (Quaternion)
        tf.transform.rotation.x = T_lidar_camera[3]
        tf.transform.rotation.y = T_lidar_camera[4]
        tf.transform.rotation.z = T_lidar_camera[5]
        tf.transform.rotation.w = T_lidar_camera[6]

        self.tf_broadcaster_.sendTransform(tf)
        self.get_logger().info("Published static transform from 'velodyne' to 'camera_frame'.")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = StaticTfBroadcaster()
        rclpy.spin(node)
    except Exception as e:
        print(f'Exception in StaticTfBroadcaster node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()