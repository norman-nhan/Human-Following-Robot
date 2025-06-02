#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

#Audio configuration
import os
import pygame 

# Import external modules
import numpy as np
from ultralytics import YOLO
# Import modules (ROS2 related)
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

# Import modules (YASMIN related)
# https://github.com/uleroboticsgroup/yasmin.git
from sensor_msgs.msg import Image
from yasmin import State, Blackboard

class ScreamingState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['target_found'])

        self.node = node

        self.image_sub = self.node.create_subscription(Image, "image_raw", self.callback, 10)
        self.cv_image = None
        self.bridge = CvBridge()

        self.audio_file = '../sounds/scream.mp3'
        self.model = YOLO('../models/best_ali.pt')
        self.mixer = pygame.mixer.init()
        os.environ["SDL_AUDIODRIVER"] = "pulseaudio"

        self.detect_log = "stop"

    def playSound(self):

        self.mixer.music.load(self.audio_file)
        self.mixer.music.play()
        self.node.get_clock().sleep_for(Duration(seconds=0.5)) # wait for 0.5 but ros node still receiving signals 

    def callback(self, msg: Image): 
        """Get a cv image."""
        self.node.get_logger().info('Receiving image')
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # pass cv image to self.cv_image
        except CvBridgeError as e:
            self.node.get_logger().error(str(e))
        return

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Screaming State")
        self.playSound()

        while self.cv_image is None and rclpy.ok():
            self.node.get_logger().info('Waiting for image')
            self.node.get_clock().sleep_for(Duration(seconds=0.5))         

        no_detection_count = 0
        results = self.model(self.cv_image)
        while (
            len(results) == 0 or
            results[0].boxes is None or
            results[0].boxes.xywh is None or
            len(results[0].boxes) == 0
        ):
            no_detection_count += 1
            if no_detection_count > 3:
                self.playSound()
            
            rclpy.spin_once(self.node)
            results = self.model(self.cv_image)
            self.node.get_clock().sleep_for(Duration(seconds=0.5))
        return 'target_found'