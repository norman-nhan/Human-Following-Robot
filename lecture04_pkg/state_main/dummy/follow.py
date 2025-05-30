#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: navigation.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# Import external modules
import numpy as np
import cv2

# Import ROS2 related modules
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Import YASMIN related modules
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import State
from yasmin import Blackboard


class FollowState(State):
    """FollowState class (inherits from State class)
    Detects red regions and follows them
    """

    def __init__(self, node: Node):
        """Class initialization method"""
        # Override the constructor of the inherited State class
        # The outcomes argument specifies the possible results to return when the state completes
        super().__init__(outcomes=["outcome"])
        self.node = node

        self.bridge = CvBridge()
        self.image_pub = self.node.create_publisher(
            msg_type=Image, topic="masked_image", qos_profile=10
        )
        self.image_sub = self.node.create_subscription(
            msg_type=Image, topic="image_raw", callback=self.callback, qos_profile=10
        )

        self.vel_pub = self.node.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

        self.cmd_vel = Twist()
        self.detect_log = "stop"

    def callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(e)

        # ========================= state =====================================
        # Red color masking process
        # =====================================================================
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, ch = hsv.shape
        hsv1 = hsv

        # Red color range 1
        hsv_min = np.array([0, 150, 150])
        hsv_max = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # Red color range 2
        hsv_min = np.array([160, 150, 150])
        hsv_max = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        # Red region mask
        mask = mask1 + mask2
        masked_hsv = cv2.bitwise_and(hsv1, hsv1, mask=mask)

        # Calculate red region
        ones = np.ones((h, w))
        masked = cv2.bitwise_and(ones, ones, mask=mask)

        # Calculate red regions in left, center, and right
        ones_left = sum(sum(masked[0:h, 0 : int(w / 3)]))
        ones_center = sum(sum(masked[0:h, int(w / 3) : int(2 * w / 3)]))
        ones_right = sum(sum(masked[0:h, int(2 * w / 3) : w]))

        # Set cmd_vel
        cmd_vel = Twist()
        if (ones_left > ones_center) and (ones_left > ones_right):
            detect_log = "Left side"
            cmd_vel.linear.x = 0.00
            cmd_vel.angular.z = 0.20
        elif (ones_center > ones_left) and (ones_center > ones_right):
            detect_log = "Center"
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = 0.00
        elif (ones_right > ones_left) and (ones_right > ones_center):
            detect_log = "Right side"
            cmd_vel.linear.x = 0.00
            cmd_vel.angular.z = -0.20
        else:
            detect_log = "stop"
            cmd_vel.linear.x = 0.00
            cmd_vel.angular.z = 0.00

        self.detect_log = detect_log
        self.cmd_vel = cmd_vel

        # Publish results
        try:
            img_cv = cv2.cvtColor(masked_hsv, cv2.COLOR_HSV2BGR)
            img_msg = self.bridge.cv2_to_imgmsg(img_cv, "bgr8")
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            self.node.get_logger().info(e)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Follow state execution method

        Args:
            blackboard (CustomBlackboard): CustomBlackboard object

        Returns:
            str: outcomes string
        """
        self.node.get_logger().info("Follow")
        self.node.get_logger().info("Start!!")
        cnt = 0
        CNT_MAX = 10
        while rclpy.ok():  # Execute loop while the node is operating correctly
            self.node.get_logger().info(self.detect_log)
            self.vel_pub.publish(self.cmd_vel)
            self.node.get_clock().sleep_for(Duration(seconds=1))

            cnt += 1
            rclpy.spin_once(self.node)
            if cnt > CNT_MAX:
                break
        # Send stop command to the robot
        self.vel_pub.publish(Twist())
        self.node.get_logger().info("Stop!!")
        self.node.get_clock().sleep_for(Duration(seconds=1))
        return "outcome"
