#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TargetMotion(Node):
    def __init__(self):
        super().__init__('target_motion')

        # Incoming teleop commands
        self.sub = self.create_subscription(
            Twist,
            '/target_cmd',
            self.cmd_callback,
            10
        )

        # Outgoing command to Gazebo diff drive plugin
        self.pub = self.create_publisher(
            Twist,
            '/target/cmd_vel',
            10
        )

        self.get_logger().info("Target motion node ready → listening on /target_cmd")

    def cmd_callback(self, msg):
        # Just forward the command to Gazebo
        self.pub.publish(msg)

        self.get_logger().info(
            f"[TARGET] linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TargetMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
