#!/usr/bin/env python3
# -*-encoding:UTF-8-*-
#Audio configuration
import os
import pygame

# Imports 
# from predict import *

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



class ScreamingState(State):
    """State class (inherits from State class)"""

    def __init__(self, node: Node):
        """Class initialization method

        Args:
            node (Node): Node class object
        """

        self.outcomes = ["Found"]

        # Override the constructor of the inherited State class
        # The outcomes argument specifies the possible results to return when the state completes
        super().__init__(outcomes=self.outcomes)

        # Create an instance of the Node object
        self.node = node
        self.image_sub = self.node.create_subscription(
            msg_type=Image, topic="image_raw", callback=self.callback, qos_profile=10
        ) # creating the subscription to the image topic with an 'inactive' node.

        self.detect_log = "stop"

        ### Internal variables ###
        
        self.audio_file = "./scream.mp3"
        self.model = YOLO("./models/best_ali.pt")
        self.detection_start_time = None

    def playSound(self):
        os.environ["SDL_AUDIODRIVER"] = "pulseaudio"

        pygame.mixer.init()
        pygame.init()

        pygame.mixer.music.load(self.audio_file)
        pygame.mixer.music.play()
        self.node.get_clock().sleep_for(Duration(seconds=3)) # wait for 0.5 seconds to give the robot time to move # TODO: 0.5 is too long ? too short ? Was 1sec in the original code


    def callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(e)

        frame = cv_image.copy()
        results = self.model(frame)[0]
        boxes = results.boxes.xyxy
        current_time = self.node.get_clock().now().nanoseconds / 1e9  # seconds

        if len(boxes) > 0:
            if self.detection_start_time is None:
                self.detection_start_time = current_time
            elif current_time - self.detection_start_time >= 5:
                self.detect_log = "Found"
        else:
            self.detection_start_time = None
            self.detect_log = "NotFound"
            self.playSound()
            return
        
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
            self.node.get_clock().sleep_for(Duration(seconds=0.5)) # wait for 0.5 seconds to give the robot time to move # TODO: 0.5 is too long ? too short ? Was 1sec in the original code
            rclpy.spin_once(self.node) # activate the node to process incoming images

            if self.detect_log in self.outcomes:
                self.node.get_logger().info(f"State outcome: {self.detect_log}")

                # Send stop command to the robot
                self.node.get_clock().sleep_for(Duration(seconds=1))
                return self.detect_log

    