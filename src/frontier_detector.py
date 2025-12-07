#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import std_msgs.msg import Bool

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.map = None

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.frontier_list_pubisher = self.create_publisher(PoseArray, 'frontier_list', 10)

        self.init_timer = self.create_timer(3.0, self.first_detect_frontiers)
        self.create_subscription(Bool, "/frontier_reached", self.reached_callback, 10)
        
    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info("Updated Map Recieved!")
        self.detect_frontiers()

    def first_detect_frontiers(self):
        if self.map is None:
            return
        self.detect_frontiers()     # generate first frontier
        self.init_timer.cancel()    # stop timer forever

    def reached_callback(self, msg):
        if msg.data:       # only trigger on True
            self.detect_frontiers()

    def detect_frontiers(self):
        if self.map is None:
            self.get_logger().error("No map detected.")
            return

        grid = np.array(self.map.data).reshape((self.map.info.height,self.map.info.width))

        frontiers = []
        # Find Frontiers
        for row in range(1, self.map.info.height):
            for col in range(1, self.map.info.width):
                if grid[row][col] == 0:
                    neighbors = grid[row-1:row+2, col-1:col+2]
                    if np.any(neighbors == -1):

                        # Convert grid to world coords
                        x = col * self.map.info.resolution + self.map.info.origin.position.x
                        y = row * self.map.info.resolution + self.map.info.origin.position.y
                        frontiers.append((x, y))

        # --- Publish as PoseArray ---
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in frontiers:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self.frontier_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(frontiers)} frontiers")

def main(args=None):
    rclpy.init(args=args)

    node = FrontierDetector()
    rclpy.spin(node)

    rclpy.shutdown()
