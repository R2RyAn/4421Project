#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time
import math

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')

        self.sub = self.create_subscription(
            Point, '/face_pixel', self.face_callback, 10)

        self.pub = self.create_publisher(
            Twist, '/pan_tilt_cmd', 10)

        # Camera resolution
        self.img_w = 640
        self.img_h = 480

        # Controller gains
        self.kp_pan = 0.0025
        self.kp_tilt = 0.0025

        # Dead-zone threshold
        self.threshold_px = 40

        # Tracking state
        self.face_seen = False
        self.last_face_time = time.time()
        self.face_timeout = 0.5

        # SMOOTH SCANNING
        self.scan_angle = 0.0           # radians
        self.scan_rate = 0.02           # rad per update (SMOOTH!)
        self.tilt_scan_angle = 0.0

        self.get_logger().info("Face tracker active → publishing to /pan_tilt_cmd")

    def face_callback(self, pt):
        now = time.time()

        # -----------------------------------
        # NO FACE DETECTED → do nothing here
        # -----------------------------------
        if pt.x < 0 or pt.y < 0:
            self.face_seen = False
            return

        # FACE FOUND
        self.face_seen = True
        self.last_face_time = now

        # NORMAL TRACKING
        cx = self.img_w / 2
        cy = self.img_h / 2

        error_pan = pt.x - cx
        error_tilt = pt.y - cy

        # Dead zone
        if abs(error_pan) < self.threshold_px:
            error_pan = 0.0
        if abs(error_tilt) < self.threshold_px:
            error_tilt = 0.0

        cmd = Twist()
        cmd.angular.z = -error_pan * self.kp_pan
        cmd.linear.x = error_tilt * self.kp_tilt

        self.pub.publish(cmd)

        #self.get_logger().info(
        #    f"[TRACKING] pan_vel={cmd.angular.z:.3f}, tilt_vel={cmd.linear.x:.3f}"
        #)

    def spin_once(self):
        now = time.time()

        # -----------------------------------
        # SMOOTH SCANNING MODE
        # -----------------------------------
        if not self.face_seen and (now - self.last_face_time) > self.face_timeout:
            
            # Increment scan angle gradually (SMOOTH)
            self.scan_angle += self.scan_rate
            if self.scan_angle > 2 * math.pi:
                self.scan_angle -= 2 * math.pi

            # Build smooth position command
            pan_position = math.sin(self.scan_angle) * 1.2   # amplitude = 1.2 rad (~70°)
            tilt_position = 0.0

            cmd = Twist()
            cmd.angular.z = pan_position     # now it's an angle, not a speed
            cmd.linear.x = tilt_position

            self.pub.publish(cmd)

            #self.get_logger().info(
            #   f"[SCANNING] sweeping_angle={pan_position:.3f} rad"
            #)

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.04)
            node.spin_once()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
