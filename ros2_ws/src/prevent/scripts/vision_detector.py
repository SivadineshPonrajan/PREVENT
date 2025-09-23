#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Vision node for TurtleBot3 waffle_pi.
# Subscribes: /camera/image_raw (sensor_msgs/Image)
# Publishes : /now_turn (std_msgs/Int32)  -> 5: left (green), 10: right (blue), 15: door (red)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

# ---------------------------
# STUDENT FUNCTION (to fill in)
# ---------------------------
def detect_dominant_arrow_color(image_bgr) -> str:
    """
    TODO (students): Return one of {"Red", "Green", "Blue", "Flame", "Unknown"} based on the image.
    Keep it lightweight and robust.
    """

    # ---------------------------
    # YOUR SOLUTION HERE (SHOULD RETURN "Red" OR "Blue" OR "Green" OR "Flame" OR "Unknown")
    # ---------------------------
    
    # ---------------------------

    return "Unknown"


class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        self.bridge = CvBridge()
        self.pub_turn = self.create_publisher(Int32, '/now_turn', 10)
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)

        # Only publish when state changes (keeps traffic light)
        self.last_cmd = 0
        self.last_pub_ns = 0
        self.REPUBLISH_NS = int(3.0 * 1e9)

        # Map color -> cue
        self.color_to_cue = {"Green": 5, "Blue": 10, "Red": 15, "Flame": 100}

        self.get_logger().info('vision_detector up (sub: /camera/image_raw, pub: /now_turn).')

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        color = detect_dominant_arrow_color(frame)
        cue = self.color_to_cue.get(color, 0)  # 0 => Unknown/no command

        # Publish only on change and only if a valid cue
        if cue != 0:
            now_ns = self.get_clock().now().nanoseconds
            # Publish on change OR if enough time has passed with the same cue
            if cue != self.last_cmd or (now_ns - self.last_pub_ns) >= self.REPUBLISH_NS:
                self.pub_turn.publish(Int32(data=cue))
                self.get_logger().info(f'Detected: {color} -> publish /now_turn: {cue}')
                self.last_cmd = cue
                self.last_pub_ns = now_ns

def main():
    rclpy.init()
    node = VisionDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
