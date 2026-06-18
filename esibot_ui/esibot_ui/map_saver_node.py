#!/usr/bin/env python3
"""
map_saver_node.py — EsiBot UI
Subscribes to /save_map (std_msgs/Empty).
Launches esibot_slam save_map.launch.py and publishes status on /save_map_status.
"""
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self.pub = self.create_publisher(String, '/save_map_status', 10)
        self.sub = self.create_subscription(Empty, '/save_map', self.on_save, 10)
        self.get_logger().info('MapSaverNode ready — listening on /save_map')

    def on_save(self, _msg):
        maps_dir = os.path.join(os.path.expanduser('~'), 'robot_ws', 'src', 'maps')
        os.makedirs(maps_dir, exist_ok=True)
        self.get_logger().info(f'Saving map to {maps_dir} ...')
        self._publish('saving')
        try:
            result = subprocess.run(
                ['ros2', 'launch', 'esibot_slam', 'save_map.launch.py',
                 f'map_dir:={maps_dir}'],
                capture_output=True, text=True, timeout=20
            )
            if result.returncode == 0:
                self.get_logger().info('Map saved')
                self._publish('saved')
            else:
                self.get_logger().error(f'Save failed: {result.stderr[:200]}')
                self._publish('error')
        except subprocess.TimeoutExpired:
            self.get_logger().error('Save timeout')
            self._publish('error')
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            self._publish('error')

    def _publish(self, status):
        msg = String()
        msg.data = status
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
