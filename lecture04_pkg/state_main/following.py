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

        # init ros node
        self.node = node

        # init cv image
        self.bridge = CvBridge()
        self.cv_image = None

        # init publishers and subscribers
        self.image_sub = self.node.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.vel_sub = self.node.create_subscription(Twist, "cmd_vel", self.vel_callback, 10)

        # init robot's velocity command
        self.cmd_vel = Twist()
        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.3

        # init model
        self.model = YOLO("../models/best_ali.pt")

        # init aruco detection
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

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
        """Get all bounding boxes from the first result."""
        assert self.cv_image is not None, "No cv image received."
        results = self.model(self.cv_image)

        if len(results) == 0 or results[0].boxes is None or results[0].boxes.xywh is None:
            return None
        
        boxes = results[0].boxes
        
        return boxes

    def get_follower_posi_and_dist(self, largest_box):
        """Get postion of follower in the image.
        
        Returns
        -------
        posi: str
            position of follower in image: 'left', 'center', 'right'.
        dist: str
            distance to follower: 'near', 'far', 'median'.

        Parameters
        ----------
        largest_box: list
            a list contains [x_center, y_center, width, height] of a bounding box
        """
        assert largest_box is not None, "No largest_box received."

        image_heigth, image_width = self.cv_image.shape[:2] # cv_image.shape[:2] gives tuple of (height, width)
        box_x_center, _ = largest_box[:2]
        if box_x_center < image_width / 3.0:
            posi = 'left'
        elif box_x_center > (2.0/3.0) * image_width:
            posi = 'right'
        else:
            posi = 'center'

        box_height = largest_box[3]
        if box_height > 0.6 * image_heigth:
            dist = "near"
        elif box_height < 0.25 * image_heigth:
            dist = "far"
        else:
            dist = "medium"

        return posi, dist

    def get_dynamic_vel(self, posi, dist) -> Twist:
        """Get the command velocity based on follower's position in image and distance to robot.
        
        Returns
        -------
        cmd_vel: Twist
        """
        assert posi is not None and dist is not None, "No follower's position and distance received."

        cmd_vel = Twist() # init a twist instance to get result

        match dist: # estimate distance between robot to follower
            case 'near':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel.linear.x - 0.01)
            case 'medium':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel.linear.x)
            case 'far':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel.linear.x + 0.01)
        match posi: # follower position in image
            case 'left':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z + 0.01)
            case 'center':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z)
            case 'right':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z - 0.01)

        return cmd_vel

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Following State")

        # count 
        no_detection_count = 0

        while rclpy.ok():
            self.node.get_logger().info(self.detect_log)

            if self.cv_image is None or self.cmd_vel is None:
                self.node.get_logger().warn('No image nor cmd_vel received.')
                continue # skip this loop
            
            ## AruCo marker Dectection ##
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            (_, ids, _) = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
            if ids is not None:
                self.detect_log = "ReachingGoal"
                self.vel_pub.publish(Twist())
                return self.detect_log

            ## (YOLO) Band bounding box Detection ##
            boxes = self.get_bounding_boxes()
            if boxes is None or boxes.xywh.shape[0] == 0:  # No boxes
                no_detection_count += 1
                if no_detection_count > 3:
                    self.detect_log = 'TargetLost'
                    self.vel_pub.publish(Twist())
                    return self.detect_log
                continue
            else:
                no_detection_count = 0 # reset no_detection_count when detected
            
            largest_box = max(boxes.xywh.cpu().numpy(), key=lambda x: x[2] * x[3]) # get the largest boudning box
            posi, dist = self.get_follower_posi_and_dist(largest_box)
            cmd_vel = self.get_dynamic_vel(posi, dist)
            self.vel_pub.publish(cmd_vel)

            # wait for 0.5 seconds then loop
            rclpy.spin_once(self.node)
            self.node.get_clock().sleep_for(Duration(seconds=0.5))