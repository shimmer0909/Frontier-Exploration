#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool


class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')
        self.frontier_list = None
        self.current_goal_active = False

        self.create_subscription(PoseArray, '/frontier_list', self.frontiers_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        self.reached_pub = self.create_publisher(Bool, "/frontier_reached", 10)
        self.current_goal_active = False
        self.min_goal_distance = 0.5  # meters

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

        except Exception as e:
            self.get_logger().warn(f"Robot pose not available: {e}")
            return None

    def frontiers_callback(self, msg):
        if self.current_goal_active:
            return

        if len(msg.poses) == 0:
            self.get_logger().info("No frontier points to explore")
            return

        frontiers = [(p.position.x, p.position.y) for p in msg.poses]
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Robot pose not available")
            return
        best_centroid = self.get_best_frontier(frontiers, robot_pose)


        if best_centroid is None:
            return

        goal_x, goal_y = best_centroid

        # don't send goals closer than 0.5 m
        goal_distance = np.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1])
        if goal_distance < self.min_goal_distance: 
            self.get_logger().info(f"Skipping very close frontier ({goal_distance:.2f} m)")
            self.reached_pub.publish(Bool(data=True))
            return

        # Publish Goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.w = 1.0

        # self.nav_client.wait_for_server()
        # self.current_goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        self.current_goal_active = True

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.current_goal_active = False
            return

        self.current_goal_active = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # def result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f"Status: {result.result}")

    #     # IMPORTANT: publish that we really reached the waypoint
    #     if hasattr(result, "result") and result.result == 0: # SUCCEEDED
    #         self.reached_pub.publish(Bool(data=True))
    #     else:
    #         self.get_logger().warn("Goal failed or status unknown — forcing next waypoint")
    #         self.reached_pub.publish(Bool(data=True))

        # self.current_goal_active = False
        # self.send_next_goal()

    def result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            self.reached_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn(f"Goal failed or unknown status ({status}) — not triggering frontier")
            # self.reached_pub.publish(Bool(data=True))

        self.current_goal_active = False


    def cluster_frontiers(self, frontier_cells):
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
                    nbr = (r+dr, c+dc)
                    if nbr in frontier_set and nbr not in visited:
                        visited.add(nbr)
                        queue.append(nbr)

            clusters.append(cluster)

        return clusters

    def get_best_frontier(self, frontiers, robot_pose):
        rx, ry = robot_pose

        # Separate close vs far
        far_frontiers = [(x, y) for (x, y) in frontiers 
                        if np.hypot(x-rx, y-ry) > self.min_goal_distance]

        if far_frontiers:
            frontiers_to_consider = far_frontiers
        else:
            frontiers_to_consider = frontiers  # if all are too close

        best_cost = None
        best_point = None

        for (x, y) in frontiers_to_consider:
            dist = np.hypot(x - rx, y - ry)
            cost = dist
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_point = (x, y)

        return best_point

def main(args=None):
    rclpy.init(args=args)

    node = FrontierSelector()
    rclpy.spin(node)

    rclpy.shutdown()

