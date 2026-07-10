import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.distance import cdist
from scipy.optimize import linear_sum_assignment

from sensor_msgs.msg import PointCloud2, PointField
import struct


# ============================================================
#  Tracker logic (UNCHANGED from sample)
# ============================================================
class PointTracker:
    def __init__(self, max_distance=2.0):
        self.next_id = 0
        self.tracks = {}  # {id: (x, y)}
        self.max_distance = max_distance

    def update(self, new_points):
        if not new_points:
            self.tracks = {}
            return {}

        if not self.tracks:
            new_tracks = {}
            for p in new_points:
                new_tracks[self.next_id] = p
                self.next_id += 1
            self.tracks = new_tracks
            return self.tracks

        track_ids = list(self.tracks.keys())
        old_coords = np.array(list(self.tracks.values()))
        new_coords = np.array(new_points)

        cost_matrix = cdist(old_coords, new_coords)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        updated_tracks = {}
        assigned_new_indices = set()

        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < self.max_distance:
                track_id = track_ids[r]
                updated_tracks[track_id] = new_points[c]
                assigned_new_indices.add(c)

        for i, p in enumerate(new_points):
            if i not in assigned_new_indices:
                updated_tracks[self.next_id] = p
                self.next_id += 1

        self.tracks = updated_tracks
        return self.tracks


# ============================================================
#  ROS2 Radar Localization Node
# ============================================================
class RadarLocalizationNode(Node):
    def __init__(self):
        super().__init__('radar_localization')

        # ---- Tracker ----
        self.tracker = PointTracker(max_distance=2.0)

        # ---- ROS Interfaces ----
        self.sub = self.create_subscription(
            PointCloud2,
            '/mr76/points_buffered',
            self.radar_callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/mr76/points_tracked',
            10
        )

        self.get_logger().info("Radar Localization Node started")

    # --------------------------------------------------------
    # Callback
    # --------------------------------------------------------
    def radar_callback(self, msg: PointCloud2):
        points = []

        step = msg.point_step
        for i in range(msg.width):
            offset = i * step
            x, y, z, _ = struct.unpack_from('fffH', msg.data, offset)
            points.append((x, y))

        # Run tracker
        tracked = self.tracker.update(points)

        # Publish tracked result
        self.publish_tracked_cloud(tracked, msg.header)

    # --------------------------------------------------------
    # Publish PointCloud2 with tracker IDs
    # --------------------------------------------------------
    def publish_tracked_cloud(self, tracked, header):
        if not tracked:
            return

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(tracked)
        cloud.is_dense = False
        cloud.is_bigendian = False

        cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='track_id', offset=12, datatype=PointField.UINT16, count=1),
        ]

        cloud.point_step = 14
        cloud.row_step = cloud.point_step * cloud.width

        cloud.data = b''.join(
            struct.pack('fffH', x, y, 0.0, tid)
            for tid, (x, y) in tracked.items()
        )

        self.pub.publish(cloud)


# ============================================================
#  main()
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = RadarLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
