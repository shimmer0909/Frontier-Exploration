#!/src/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import math
import time

STOP_THRESHOLD = 0.01      # m/s and rad/s
STABLE_TIME = 1.0          # seconds robot must be stopped

class SlamStateManager(Node):
    def __init__(self):
        super().__init__('slam_state_manager')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.pause_client = self.create_client(Empty, '/slam_toolbox/pause')
        self.resume_client = self.create_client(Empty, '/slam_toolbox/resume')

        self.last_moving_time = time.time()
        self.slam_paused = False

        self.get_logger().info("SLAM State Manager started")

    def odom_callback(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        speed = math.sqrt(linear.x**2 + linear.y**2)
        rotation = abs(angular.z)

        moving = speed > STOP_THRESHOLD or rotation > STOP_THRESHOLD

        now = time.time()

        if moving:
            self.last_moving_time = now
            if self.slam_paused:
                self.resume_slam()
        else:
            if not self.slam_paused and (now - self.last_moving_time) > STABLE_TIME:
                self.pause_slam()

    def pause_slam(self):
        if not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Pause service not available")
            return

        self.pause_client.call_async(Empty())
        self.slam_paused = True
        self.get_logger().info("SLAM paused")

    def resume_slam(self):
        if not self.resume_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Resume service not available")
            return

        self.resume_client.call_async(Empty())
        self.slam_paused = False
        self.get_logger().info("SLAM resumed")


def main():
    rclpy.init()
    node = SlamStateManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

