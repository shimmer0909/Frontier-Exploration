#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigationToPose
from std_msgs.msg import Bool


class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')
        self.frontier_list = None
        self.current_goal_active = False

        self.create_subscription(PoseArray, '/frontier_list', self.frontiers_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigationToPose, 'navigation_to_pose')

        self.reached_pub = self.create_publisher(Bool, "/frontier_reached", 10)

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)

        except Exception:
            return None

    def frontiers_callback(self, msg):
        if len(msg.pose) == 0:
            self.get_logger().info("No frontier points to explore")
            return

        frontiers = [(p.position.x, p.position.y) for p in msg.poses]
        frontier_clusters = self.cluster_frontiers(frontiers)
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Robot pose not available")
            return
        best_cluster, best_centroid = self.get_best_frontier(frontier_clusters, robot_pose)

        if best_centroid is None:
            return

        goal_x, goal_y = best_centroid

        # Publish Goal
        goal = NavigationToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        # self.current_goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            # self.current_goal_active = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Status: {result.result}")

        # IMPORTANT: publish that we really reached the waypoint
        if hasattr(result, "result") and result.result == 0: # SUCCEEDED
            self.reached_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn("Goal failed or status unknown — forcing next waypoint")
            self.reached_pub.publish(Bool(data=True))

        # self.current_goal_active = False
        # self.send_next_goal()


    def cluster_frontiers(self, frontier_list):
        if len(frontier_list) == 0:
            return []
            
        # frontier_set = set(frontier_list)
        visited = set()

        clusters = []

        # neighbour_directions = [(-1, -1), (-1, 0), (-1, 1),
        #             (0, -1), (0, 1),
        #             (1, -1), (1, 0), (1, 1)]

        for cell in frontier_list:
            if cell in visited:
                continue

            cluster = []
            queue = [cell]
            visited.add(cell)

            while queue:
                # row,col = queue.pop(0)
                # cluster.append((row,col))

                # for dr, dc in neighbour_directions:
                #     nr, nc = row + dr, col + dc
                #     neighbour = (nr, nc)

                #     if neighbour not in visited and neighbour in frontier_set:
                #         visited.add(neighbour)
                #         queue.append(neighbour)

                x, y = queue.pop(0)
                cluster.append((x, y))

                for nx, ny in frontier_list:
                    if (nx, ny) not in visited:
                        if np.hypot(nx-x, ny-y) < 0.5:  # ~1 grid cell
                            visited.add((nx, ny))
                            queue.append((nx, ny))

            clusters.append(cluster)

        return clusters

    def get_best_frontier(self, clusters, robot_pose):
        if not clusters:
            return None, None
        
        rx, ry = robot_pose

        best_score = float('inf')
        best_cluster = None
        best_centroid = None

        for cluster in clusters:
            rows = [p[0] for p in cluster]
            cols = [p[1] for p in cluster]

            cx = np.mean(rows)
            cy = np.mean(cols)

            dist = np.hypot(cx-rx, cy-ry)

            size = len(cluster)

            cost = dist - 0.2*size

            if cost < best_score:
                best_score = cost
                best_cluster = cluster
                best_centroid = (cx,cy)

        return best_cluster, best_centroid

def main(args=None):
    rclpy.init(args=args)

    node = FrontierSelector()
    rclpy.spin(node)

    rclpy.shutdown()

