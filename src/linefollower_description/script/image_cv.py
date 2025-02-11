#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follower_node")
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.camera_callback, 10)
        self.publisher_ = self.create_publisher(TwistStamped,"/linefollower_controller/cmd_vel",10)
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        self.get_logger().info("Received camera frame")
        msg_twist=TwistStamped()
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, threshold = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
        
        height, width = threshold.shape
        roi = threshold[int(height / 2):, :]
        
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                deviation = cx - (width // 2)
                self.get_logger().info(f"Deviation: {deviation}")
                if deviation < -100:
                    msg_twist.twist.angular.z=0.5  
                elif deviation > 100:
                    msg_twist.twist.angular.z=-0.5
                else:
                    msg_twist.twist.linear.x=0.1
            else:
                msg_twist.twist.linear.x=0.0
                msg_twist.twist.angular.z=0.0
        else:
            msg_twist.twist.linear.x=0.0
            msg_twist.twist.angular.z=0.0
        self.publisher_.publish(msg_twist)
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":  # Corrected this line
    main()
