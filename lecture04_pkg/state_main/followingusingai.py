#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

# Import external modules
import numpy as np
import cv2, cv2.aruco
from ultralytics import YOLO
# Import modules (ROS2 related)
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

# Import modules (YASMIN related)
# https://github.com/uleroboticsgroup/yasmin.git
from sensor_msgs.msg import Image
from yasmin import State
from yasmin import Blackboard


class FollowingState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=["ReachingGoal", "TargetLost"])

        self.node = node
        self.bridge = CvBridge()
        self.image_pub = self.node.create_publisher(Image, "masked_image", 10)
        self.image_sub = self.node.create_subscription(Image, "image_raw", self.callback, 10)
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)

        self.cmd_vel = Twist()
        self.detect_log = "stop"

        self.model = YOLO("./models/best_ali.pt")
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.targetSpeed = 0.05
        self.detection_start_time = None
        self.tracking_active = False

    def callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(str(e))
            return

        frame = cv_image.copy()
        results = self.model(frame)[0]
        boxes = results.boxes.xyxy

        frame_h, frame_w = frame.shape[:2]
        current_time = self.node.get_clock().now().nanoseconds / 1e9  # seconds

        cmd_vel = Twist()
        detect_log = "stop"

        if len(boxes) > 0:
            if self.detection_start_time is None:
                self.detection_start_time = current_time
            elif current_time - self.detection_start_time >= 5:
                self.tracking_active = True
        else:
            self.detection_start_time = None
            self.tracking_active = False
            self.cmd_vel = Twist()
            self.detect_log = "TargetLost"
            return

        if self.tracking_active:
            # Use largest box (biggest area)
            largest_box = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
            x1, y1, x2, y2 = map(int, largest_box)
            center_x = (x1 + x2) // 2
            rel_height = (y2 - y1) / frame_h
            object_area = (x2 - x1) * (y2 - y1)

            # Distance classification
            if rel_height > 0.6:
                distance = "Near"
            elif rel_height < 0.25:
                distance = "Far"
            else:
                distance = "Medium"

            # Position classification
            if center_x < frame_w / 3:
                pos = "left"
                cmd_vel.angular.z = 0.2
            elif center_x > 2 * frame_w * 2 / 3:
                pos = "right"
                cmd_vel.angular.z = -0.2
            else:
                pos = "center"
                cmd_vel.angular.z = 0.0

            # Movement logic
            if distance == "Medium" or distance == "Near":
                detect_log = "stop"
                cmd_vel.linear.x = 0.0
            else:
                detect_log = f"follow, {pos}"
                if self.targetSpeed > cmd_vel.linear.x:
                    cmd_vel.linear.x = min(self.targetSpeed, cmd_vel.linear.x + 0.005)
                else:
                    cmd_vel.linear.x = max(self.targetSpeed, cmd_vel.linear.x - 0.005)

        # ArUco detection (stop override)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (_, ids, _) = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        if ids is not None:
            detect_log = "ReachingGoal"
            cmd_vel.linear.x = max(0.00, cmd_vel.linear.x - 0.01)
            cmd_vel.angular.z = 0.00

        # Publish masked image (optional: color processing)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            self.node.get_logger().info(str(e))

        self.detect_log = detect_log
        self.cmd_vel = cmd_vel

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing state Following")
        self.color = blackboard.color

        while rclpy.ok():
            self.node.get_logger().info(self.detect_log)
            self.vel_pub.publish(self.cmd_vel)
            self.node.get_clock().sleep_for(Duration(seconds=0.5))
            rclpy.spin_once(self.node)

            if self.detect_log in self.outcomes:
                self.node.get_logger().info(f"State outcome: {self.detect_log}")
                self.vel_pub.publish(Twist())
                self.node.get_clock().sleep_for(Duration(seconds=1))
                return self.detect_log
