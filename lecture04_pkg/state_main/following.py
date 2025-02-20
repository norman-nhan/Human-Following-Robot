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
        self.image_sub = self.node.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.vel_pub = self.node.create_subscription(Twist, "cmd_vel", self.vel_callback, 10)

        self.cmd_vel = Twist()
        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.3

        self.model = YOLO("../models/best_ali.pt")
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_detector_default_params = cv2.aruco.DetectorParameters_create()

        self.detect_log = "Initializing"

    def vel_callback(self, msg: Twist):
        """Get robot linear, angular velocity"""
        self.cmd_vel = msg
        return
    
    def image_callback(self, msg: Image): 
        """Get a cv image."""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # pass cv image to self.cv_image
        except CvBridgeError as e:
            self.node.get_logger().info(str(e))
            return

    def get_bounding_boxes(self):
        """Get x_center, y_center, width and height of bounding boxes from the first result.
        
        Returns
        -------
        Tensor or None 
        """
        # predict
        results = self.model(self.cv_image)
        if results is None:
            return None
        else:
            return results[0].boxes.xywh # get 1st result's boxes

    def get_follower_posi_and_dist(self) -> str:
        """Get postion of follower in the image.
        
        Returns
        -------
        follower_position: str
            position of follower in image: 'left', 'center', 'right', 'out_of_view'.
        dist2follower: str
            distance to follower: 'near', 'far', 'median'
        """
        largest_box = self.get_largest_box()
        if largest_box is None:
            follower_position = 'out_of_view'
        
        _, image_width = self.cv_image.shape[:2] # cv_image.shape[:2] gives tuple of (height, width)
        box_x_center, _ = largest_box[:2]
        if box_x_center < image_width / 3.0:
            follower_position = 'left'
        elif box_x_center > (2.0/3.0) * image_width:
            follower_position = 'right'
        else:
            follower_position = 'center'

        box_height = largest_box[3]
        if box_height > 0.6:
            dist2follower = "near"
        elif box_height < 0.25:
            dist2follower = "far"
        else:
            dist2follower = "medium"

        return follower_position, dist2follower

    def get_dynamic_vel(self) -> Twist:
        """Get the command velocity based on distance to follower.
        
        Returns
        -------
        cmd_vel: Twist
        """
        cmd_vel = Twist() # init a twist instance to get result

        posi, dist = self.get_follower_posi_and_dist()
        if dist is None:
            self.node.get_logger().warn("Cannot receive any position and distance of follower")
            return None
        
        match posi:
            case 'near':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel - 0.01)
            case 'medium':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel)
            case 'far':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel + 0.01)
        match:
            case 'left':
                cmd_vel.angular
        return cmd_vel

    def get_largest_box(self):
        """Get the biggest bouding box"""
        boxes = self.get_bounding_boxes()
        if boxes is None:
            self.node.get_logger().info("None bounding boxes received")
            return None
        
        ### boxes structure ###
        # [x_center, y_center, width, height]
        return max(boxes, key=lambda x: x[2] * x[3]) # get the biggest area boudning box

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Following State")

        while rclpy.ok():
            self.node.get_logger().info(self.detect_log)
        #     self.vel_pub.publish(self.cmd_vel)
        #     self.node.get_clock().sleep_for(Duration(seconds=0.5))
        #     rclpy.spin_once(self.node)

        #     if self.detect_log in self.outcomes:
        #         self.node.get_logger().info(f"State outcome: {self.detect_log}")
        #         self.vel_pub.publish(Twist())
        #         self.node.get_clock().sleep_for(Duration(seconds=1))
        #         return self.detect_log

        # results = self.model(self.cv_image) # get the result from YOLO model
        # # check for an amount of time if we don't have any result 
        # if results is not None:
        #     boxes = results[0].boxes.xyxy # get the 1st result's boxes

        #     image_h, image_w = self.cv_image.shape[:2]

        #     if self.tracking_active:
        #     # ArUco detection (stop override)
        #     gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #     (_, ids, _) = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        #     if ids is not None:
        #         detect_log = "ReachingGoal"
        #         cmd_vel.linear.x = max(0.00, cmd_vel.linear.x - 0.01)
        #         cmd_vel.angular.z = 0.00

        #     # Publish masked image (optional: color processing)
        #     try:
        #         img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        #         self.image_pub.publish(img_msg)
        #     except CvBridgeError as e:
        #         self.node.get_logger().info(str(e))

        #     self.detect_log = detect_log
        #     self.cmd_vel = cmd_vel
