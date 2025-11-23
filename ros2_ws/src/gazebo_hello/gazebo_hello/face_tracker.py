#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')

        # Listen to face pixel coordinates
        self.sub = self.create_subscription(
            Point, '/face_pixel', self.face_callback, 10)

        # Publish to the same topic teleop uses
        self.pub = self.create_publisher(
            Twist, '/pan_tilt_cmd', 10)

        # Camera resolution
        self.img_w = 640
        self.img_h = 480

        # Controller gains
        self.kp_pan = 0.0025
        self.kp_tilt = 0.0025

        self.get_logger().info("Face tracker active → publishing to /pan_tilt_cmd")

    def face_callback(self, pt):
        cx = self.img_w / 2
        cy = self.img_h / 2

        # Pixel error
        error_pan = pt.x - cx     # left-right
        error_tilt = pt.y - cy    # up-down

        # Build Twist cmd for teleop controller
        cmd = Twist()
        cmd.angular.z = -error_pan * self.kp_pan   # pan
        cmd.linear.x = error_tilt * self.kp_tilt   # tilt

        self.pub.publish(cmd)

        self.get_logger().info(
            f"Tracking → pan_cmd={cmd.angular.z:.3f}, tilt_cmd={cmd.linear.x:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
