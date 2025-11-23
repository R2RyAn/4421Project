#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')

        self.bridge = CvBridge()

        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.pub = self.create_publisher(Point, '/face_pixel', 10)

        self.get_logger().info("Face detector ready")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )

        if len(faces) == 0:
            return

        (x, y, w, h) = max(faces, key=lambda f: f[2] * f[3])
        cx = x + w // 2
        cy = y + h // 2

        pt = Point()
        pt.x = float(cx)
        pt.y = float(cy)

        self.pub.publish(pt)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
