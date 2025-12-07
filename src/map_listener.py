#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapListener(Node):
    def __init__(self):
        super().__init__('map_listener')
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info("Map updated")

def main(args=None):
    rclpy.init(args=args)

    node = MapListener()
    rclpy.spin(node)

    rclpy.shutdown()