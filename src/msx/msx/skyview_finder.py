#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from driving_swarm_utils.node import DrivingSwarmNode, main_fn

class SkyviewFinder(DrivingSwarmNode):
    def __init__(self):
        super().__init__('skyviewFinder')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            10
        )

        # === Publisher für Objektpositionen ===
        self.pub_pos = self.create_publisher(
            Float64MultiArray,
            "/skyview_robot_pos",
            10
        )

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        img_h, img_w = frame.shape[:2]

        detected_positions = []   # sammelt [x, y]

        # --- ROTE OBJEKTE ---
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                detected_positions.append(float(cx))
                detected_positions.append(float(cy))

                self.get_logger().info(f"Red object at: x={cx}, y={cy}")

        # --- DUNKLE OBJEKTE ---
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([179, 50, 120])
        mask_dark = cv2.inRange(hsv, lower_dark, upper_dark)

        contours, _ = cv2.findContours(mask_dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            (x, y), radius = cv2.minEnclosingCircle(cnt)

            if area > 5:
                circle_area = np.pi * radius * radius
                ratio = area / circle_area
                if 0.5 < ratio < 1.5:
                    cx, cy = int(x), int(y)

                    detected_positions.append(float(cx))
                    detected_positions.append(float(cy))

                    self.get_logger().info(f"Circle object at: x={cx}, y={cy}")

        # === Publish detected positions ===
        msg_out = Float64MultiArray()
        msg_out.data = detected_positions
        self.pub_pos.publish(msg_out)

        # --- Visualization optional ---
        mask = mask_red | mask_dark
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("RoofCam Original", frame)
        cv2.imshow("Red-Mask", mask_red)
        cv2.imshow("Dark Mask", mask_dark)
        cv2.imshow("Found", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SkyviewFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
