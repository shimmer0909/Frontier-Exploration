#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.map = None

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.frontier_list_pubisher = self.create_publisher(PoseArray, 'frontier_list', 10)

        self.init_timer = self.create_timer(3.0, self.first_detect_frontiers)
        self.create_subscription(Bool, "/frontier_reached", self.reached_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.min_goal_distance = 0.5  # meters
        
    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info("Updated Map Recieved!")
        # self.detect_frontiers()

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"Robot pose not available: {e}")
            return None


    def first_detect_frontiers(self):
        if self.map is None:
            return
        self.detect_frontiers()     # generate first frontier
        self.init_timer.cancel()    # stop timer forever

    def reached_callback(self, msg):
        if msg.data:       # only trigger on True
            self.detect_frontiers()

    def cluster_frontiers_grid(self, frontier_cells):
        frontier_set = set(frontier_cells)
        visited = set()
        clusters = []

        neighbors = [(-1,-1), (-1,0), (-1,1),
                    (0,-1),          (0,1),
                    (1,-1),  (1,0),  (1,1)]

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            queue = [cell]
            visited.add(cell)

            while queue:
                r, c = queue.pop(0)
                cluster.append((r, c))

                for dr, dc in neighbors:
                    nbr = (r + dr, c + dc)
                    if nbr in frontier_set and nbr not in visited:
                        visited.add(nbr)
                        queue.append(nbr)

            clusters.append(cluster)

        return clusters

    def detect_frontiers(self):
        if self.map is None:
            self.get_logger().error("No map detected.")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().error("Robot Pose is None.")
            return
        rx, ry = robot_pose


        grid = np.array(self.map.data).reshape((self.map.info.height,self.map.info.width))

        frontiers = []
        # Find Frontiers
        for row in range(1, self.map.info.height-1):
            for col in range(1, self.map.info.width-1):
                if grid[row][col] == 0:
                    neighbors = grid[row-1:row+2, col-1:col+2]
                    if np.any(neighbors == -1):

                        # Convert grid to world coords
                        x = col * self.map.info.resolution + self.map.info.origin.position.x
                        y = row * self.map.info.resolution + self.map.info.origin.position.y
                        if np.hypot(x - rx, y - ry) >= self.min_goal_distance:
                            frontiers.append((row, col))

        clusters = self.cluster_frontiers_grid(frontiers)

        # --- Publish as PoseArray ---
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for cluster in clusters:
            if len(cluster) < 5:
                continue

            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]

            mean_row = np.mean(rows)
            mean_col = np.mean(cols)

            # convert centroid to world coords
            x = mean_col * self.map.info.resolution + self.map.info.origin.position.x
            y = mean_row * self.map.info.resolution + self.map.info.origin.position.y

            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self.frontier_list_pubisher.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} frontier clusters")

def main(args=None):
    rclpy.init(args=args)

    node = FrontierDetector()
    rclpy.spin(node)

    rclpy.shutdown()
