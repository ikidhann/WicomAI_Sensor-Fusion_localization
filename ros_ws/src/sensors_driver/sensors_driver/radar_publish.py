import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
from sensors_processing.Radar_MR76 import RadarMR76

class RadarPublisher(Node):
    def __init__(self):
        super().__init__('radar_publisher')

        # ---------- Parameters ----------
        self.device = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.frame_id = 'radar_link'

        self.frames_to_accumulate = 1 # number of frames to buffer
        self.current_frame_count = 0

        # ---------- Radar ----------
        self.radar_conn = RadarMR76(self.device, self.baudrate, 1)

        # ---------- Buffer ----------
        self.point_buffer = []   # list of (x, y, z)

        # ---------- ROS ----------
        self.publisher = self.create_publisher(
            PointCloud2,
            '/mr76/points_buffered',
            10
        )

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info("MR76 RadarPublisher started")

    def timer_callback(self):
        data_bit = self.radar_conn.read()
        if data_bit is None:
            return

        # ---- Frame header ----
        if data_bit.startswith("60a"):
            result = self.radar_conn.parse_info(data_bit)
            self.radar_conn.tot_det_obj = int(result[0])
            self.radar_conn.cur_len_obj = 0
            self.radar_conn.obj_list = []
            return

        # ---- Object packet ----
        if data_bit.startswith("60b") and \
           self.radar_conn.cur_len_obj < self.radar_conn.tot_det_obj:

            obj = self.radar_conn.parse_target_info(data_bit)
            self.radar_conn.obj_list.append(obj)
            self.radar_conn.cur_len_obj += 1

        # ---- Full frame collected ----
        if len(self.radar_conn.obj_list) == self.radar_conn.tot_det_obj and \
           self.radar_conn.tot_det_obj > 0:

            # 1 objects → 1 point
            for o in self.radar_conn.obj_list:
                x = o['dist_long']
                y = o['dist_lat']
                z = 0.0
                self.point_buffer.append((x, y, z))

            self.current_frame_count += 1

            # reset per-frame storage
            self.radar_conn.obj_list = []
            self.radar_conn.cur_len_obj = 0

        # ---- Publish buffered cloud ----
        if self.current_frame_count >= self.frames_to_accumulate:
            self.publish_buffered_cloud()
            self.point_buffer = []
            self.current_frame_count = 0

    def publish_buffered_cloud(self):
        if len(self.point_buffer) == 0:
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(self.point_buffer)
        msg.is_dense = False
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = b''.join(
            struct.pack('fff', *p) for p in self.point_buffer
        )

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published buffered cloud with {msg.width} points"
            #f"Published points"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RadarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#update