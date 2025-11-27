#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TargetMotion(Node):
    def __init__(self):
        super().__init__('target_motion')

        # Publish directly to the same Gazebo joint trajectory interface
        self.pub = self.create_publisher(
            JointTrajectory,
            '/set_joint_trajectory',
            10
        )

        # Internal state for the prismatic joints
        self.z_pos = 0.0   # for z_joint
        self.x_pos = 0.0   # for x_joint

        # Step sizes per update (in meters)
        self.z_step = 0.05
        self.x_step = 0.05

        # Clamp to safe range (subset of your URDF limits)
        self.z_min = -0.5
        self.z_max = 0.5
        self.x_min = -0.5
        self.x_max = 0.5

        # Timer to update motion periodically
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(
            "Random target motion node running → publishing z_joint/x_joint to /set_joint_trajectory"
        )

    def timer_callback(self):
        # Random small change for each joint
        dz = random.uniform(-self.z_step, self.z_step)
        dx = random.uniform(-self.x_step, self.x_step)

        self.z_pos = max(self.z_min, min(self.z_max, self.z_pos + dz))
        self.x_pos = max(self.x_min, min(self.x_max, self.x_pos + dx))

        # Build JointTrajectory for ONLY the target joints
        traj = JointTrajectory()
        traj.header.frame_id = "world"  # frame doesn't really matter for this plugin

        # IMPORTANT: must match your URDF joint names exactly
        traj.joint_names = ["z_joint", "x_joint"]

        point = JointTrajectoryPoint()
        point.positions = [self.z_pos, self.x_pos]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        traj.points = [point]

        self.pub.publish(traj)

        # Uncomment if you want debug logs
        # self.get_logger().info(
        #     f"[TARGET] z_joint={self.z_pos:.2f}, x_joint={self.x_pos:.2f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = TargetMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
