#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PanTiltTeleop(Node):
    def __init__(self):
        super().__init__('pan_tilt_teleop')

        # Publisher to the Gazebo interface
        self.pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        # Subscribe to teleop twist
        self.sub = self.create_subscription(
            Twist,
            '/pan_tilt_cmd',
            self.cmd_callback,
            10
        )

        # State
        self.pan = 0.0
        self.tilt = 0.0
        self.scale_pan = 0.05     # how fast joystick changes angle
        self.scale_tilt = 0.05

        self.get_logger().info("Pan-Tilt teleop ready. Use teleop_twist_keyboard!")

    def cmd_callback(self, msg: Twist):
        # Map Twist → Pan/Tilt
        d_pan = msg.angular.z * self.scale_pan
        d_tilt = msg.linear.x * self.scale_tilt

        self.pan += d_pan
        self.tilt += d_tilt

        # Build trajectory message
        traj = JointTrajectory()
        traj.header.frame_id = "base_link"   # IMPORTANT!

        traj.joint_names = ["pan_joint", "tilt_joint"]

        point = JointTrajectoryPoint()
        point.positions = [self.pan, self.tilt]
        point.time_from_start.sec = 1

        traj.points = [point]

        # Publish
        self.pub.publish(traj)

        self.get_logger().info(
            f"Pan: {self.pan:.2f}, Tilt: {self.tilt:.2f} "
            f"(input pan={msg.angular.z:.2f}, tilt={msg.linear.x:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
