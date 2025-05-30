#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

# Imports 
from predict import *

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
    """FollowingState class (inherits from State class)"""

    def __init__(self, node: Node):
        """Class initialization method

        Args:
            node (Node): Node class object
        """

        self.outcomes = ["QRFound", "TargetLost"]

        # Override the constructor of the inherited State class
        # The outcomes argument specifies the possible results to return when the state completes
        super().__init__(outcomes=self.outcomes)

        # Create an instance of the Node object
        self.node = node
        self.bridge = CvBridge()
        self.image_pub = self.node.create_publisher(
            msg_type=Image, topic="masked_image", qos_profile=10
        ) # creating the publisher to the masked image topic (debug purpose)

        self.image_sub = self.node.create_subscription(
            msg_type=Image, topic="image_raw", callback=self.callback, qos_profile=10
        ) # creating the subscription to the image topic with an 'inactive' node.

        self.vel_pub = self.node.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10) # to give the robot velocity command

        self.cmd_vel = Twist()
        self.detect_log = "stop"

        ### Internal variables ###

        self.model = YOLO("../best.pt")

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.targetSpeed = 0.05 # goal speed, to be reached smoothly #TODO: the same for angular speed too ?

    def callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(e)

        if self.detect_log == "ReachingGoal": # To stop the robot smoothly
            cmd_vel.linear.x = max(0.00, cmd_vel.linear.x - 0.01)
            if cmd_vel.linear.x < 0.01:
                detect_log = "QRFound"
            return

        
        results = self.model(frame)

        objectId = ... # find the biggest one or something with a remembered color ???
        objectArea = ...
        objectCenterLocation = ...

        cmd_vel = Twist()

        # TODO: better adaptative speeds ?
        # TODO: better lost condition ?

        # We compare the number of 1 in each area.
        threshold_min = 0.1 * h * w  # 10% of the image size, we are too far from the target
        threshold_max = 0.4 * h * w  # 40% of the image size, we are too close to the target

        # TODO: test if it creates jolting effect when the robot is moving ?
        if self.targetSpeed > cmd_vel.linear.x:
            cmd_vel.linear.x = min(self.targetSpeed, cmd_vel.linear.x + 0.005)
        else:
            cmd_vel.linear.x = max(self.targetSpeed, cmd_vel.linear.x - 0.005)

        if objectArea < threshold_min:
            # we are too far from the target
            detect_log = "TargetLost"
            cmd_vel.linear.x = 0.00 # can't be smooth : we have to stop immediately if the nurse is lost
            cmd_vel.angular.z = 0.00

        if #objectCenterLocation on the left:
            detect_log = "LeftSide"
            cmd_vel.angular.z = 0.20
        elif #objectCenterLocation on the left:
            detect_log = "RightSide"
            cmd_vel.angular.z = -0.20  
        else:
            detect_log = "Center"
            cmd_vel.angular.z = 0.00

        if objectArea > threshold_max:
            # we are too close to the target
            detect_log = "Stop"
            cmd_vel.linear.x = cmd_vel.linear.x - 0.01
            cmd_vel.angular.z = 0.00

        self.detect_log = detect_log
        self.cmd_vel = cmd_vel # Is published in the execute method

        ### ArUco detection ###
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        (_, ids, _) = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        if ids is not None:
            # If the marker is detected, we stop the robot smoothly
            detect_log = "ReachingGoal"
            cmd_vel.linear.x = max(0.00, cmd_vel.linear.x - 0.01)
            cmd_vel.angular.z = 0.00

        # Publish masked image
        try:
            img_cv = cv2.cvtColor(masked_image, cv2.COLOR_HSV2BGR)
            img_msg = self.bridge.cv2_to_imgmsg(img_cv, "bgr8")
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            self.node.get_logger().info(e)

    def execute(self, blackboard: Blackboard) -> str:
        """
        Following state execution method

        Args:
            blackboard (CustomBlackboard): CustomBlackboard object

        Returns:
            str: outcomes string
        """
        # Display log
        self.node.get_logger().info("Executing state Following")

        self.color = blackboard.color  # Get the color from the blackboard

        while rclpy.ok():  # Execute loop while the node is operating correctly
            self.node.get_logger().info(self.detect_log) # log the detection status
            self.vel_pub.publish(self.cmd_vel)
            self.node.get_clock().sleep_for(Duration(seconds=0.5)) # wait for 0.5 seconds to give the robot time to move # TODO: 0.5 is too long ? too short ? Was 1sec in the original code
            rclpy.spin_once(self.node) # activate the node to process incoming images

            if self.detect_log in self.outcomes:
                self.node.get_logger().info(f"State outcome: {self.detect_log}")

                # Send stop command to the robot
                self.vel_pub.publish(Twist())
                self.node.get_clock().sleep_for(Duration(seconds=1))
                return self.detect_log

    