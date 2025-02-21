#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

class MapSwitcher(Node):
    def __init__(self):
        super().__init__('map_switcher')
        self.cli = self.create_client(LoadMap, '/map_server/load_map')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Waiting for /map_server/load_map service...')

    def switch_map(self, map_yaml_path):
        req = LoadMap.Request()
        req.map_url = map_yaml_path
        future = self.cli.call_async(req)

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Successfully switched to map: {map_yaml_path}")
        else:
            self.get_logger().error(f"Failed to switch to map: {map_yaml_path}")

def main():
    rclpy.init()
    node = MapSwitcher()
    
    # Change this path to the actual YAML map file you want to switch to
    map_path = "/home/binesh/turtle_ws/src/multi_map_navigation/maps/mappp.yaml"
    
    node.switch_map(map_path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
