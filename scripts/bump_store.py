#!/usr/bin/env python3
"""Keep the bumps motion_check marks, beside the saved map, for the costmaps.

Runs on the PC, started by pc_bringup.launch.py. motion_check runs on the car
and publishes each new row of marks on /motion_check/new_bumps (in the map
frame) when the car gets stuck against something the lidar can't see. This
adds them to the list, saves it to bumps_file, and publishes the whole list
on /motion_check/bumps (PointCloud2, about once a second), which the
costmaps' bump_layer (config/nav2.yaml) turns into obstacles that the lidar
doesn't clear, so Nav2 plans around them.

Bumps belong to a map frame, so the file sits beside the saved map in use,
and is loaded at start if load_bumps is true. /motion_check/clear_bumps
(std_srvs/Empty) forgets them all and clears the costmaps.
"""
import os

import rclpy
import rclpy.executors
import yaml
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Empty

BUMP_HEIGHT = 0.05     # m: z of the points, within the costmap's heights


class BumpStore(Node):
    def __init__(self):
        super().__init__('bump_store')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.bumps_file = os.path.expanduser(self.declare_parameter(
            'bumps_file', '~/.ros/car_s_bumps.yaml').value)
        self.bumps = []       # [x, y] in map_frame
        if self.declare_parameter('load_bumps', False).value and os.path.exists(self.bumps_file):
            with open(self.bumps_file) as f:
                self.bumps = (yaml.safe_load(f) or {}).get('bumps', [])
            self.get_logger().info(f'Loaded {len(self.bumps)} bump points from {self.bumps_file}')
        else:
            self.save_bumps()

        self.bumps_pub = self.create_publisher(PointCloud2, '/motion_check/bumps', 10)
        self.create_subscription(PointCloud2, '/motion_check/new_bumps', self.on_new_bumps, 10)
        self.clear_clients = [self.create_client(ClearEntireCostmap, name) for name in (
            '/local_costmap/clear_entirely_local_costmap',
            '/global_costmap/clear_entirely_global_costmap')]
        self.create_service(Empty, '/motion_check/clear_bumps', self.on_clear_bumps)
        self.create_timer(1.0, self.publish_bumps)

    def on_new_bumps(self, msg):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'Ignoring bumps in {msg.header.frame_id}, not {self.map_frame}')
            return
        new = [[round(float(x), 3), round(float(y), 3)]
               for x, y in point_cloud2.read_points(msg, field_names=('x', 'y'))]
        self.bumps += new
        self.get_logger().info(f'{len(new)} bump points marked, {len(self.bumps)} in all')
        self.save_bumps()
        self.publish_bumps()

    def save_bumps(self):
        try:
            os.makedirs(os.path.dirname(self.bumps_file) or '.', exist_ok=True)
            with open(self.bumps_file, 'w') as f:
                yaml.safe_dump({'frame': self.map_frame, 'bumps': self.bumps}, f)
        except OSError as e:
            self.get_logger().warning(f'Can\'t save the bumps to {self.bumps_file}: {e}')

    def publish_bumps(self):
        header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        points = [(p[0], p[1], BUMP_HEIGHT) for p in self.bumps]
        self.bumps_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def on_clear_bumps(self, request, response):
        self.get_logger().info(f'Clearing {len(self.bumps)} bump points')
        self.bumps = []
        self.save_bumps()
        self.publish_bumps()
        for client in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
        return response


def main():
    rclpy.init()
    node = BumpStore()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
