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
        super().__init__(outcomes=['exit', 'target_found'])

        self.node = node

        self.image_sub = self.node.create_subscription(Image, "image_raw", self.callback, 10)
        self.cv_image = None
        self.bridge = CvBridge()

        self.audio_file = 'lecture04_pkg/sounds/scream.mp3'
        self.model = YOLO('/home/ros2/ros2_lecture_ws/src/7_lectures/lecture04_pkg/lecture04_pkg/models/best_krisna2.pt')
        pygame.mixer.init()
        os.environ["SDL_AUDIODRIVER"] = "pulseaudio"

        self.target_found = False

    def playSound(self):
        pygame.mixer.music.load(self.audio_file)
        pygame.mixer.music.play()
        self.node.get_clock().sleep_for(Duration(seconds=2)) # wait for 0.5 but ros node still receiving signals 

    def callback(self, msg: Image): 
        """Get a cv image."""
        self.node.get_logger().debug(f'{self.__class__.__name__}: Receiving image')
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # pass cv image to self.cv_image
        except CvBridgeError as e:
            self.node.get_logger().error(str(e))

    def get_krisna_model_result(self, results):
        boxes_data = results[0].boxes.data
        boxes_xywh = results[0].boxes.xywh 
        mask = boxes_data[:, 5] == 0
        boxes = boxes_xywh[mask]

        return boxes

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing Screaming State")

        while self.cv_image is None and rclpy.ok():
            self.node.get_logger().info('Waiting for image...')
            rclpy.spin_once(self.node, timeout_sec=0.1)
            continue

        no_detection_count = 0
        while self.target_found != True: # looping if target is not found
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # YOLO detection
            results = self.model(self.cv_image, conf=0.6)
            self.rendered_image = results[0].plot()
            boxes = self.get_krisna_model_result(results)
            np_boxes = boxes.cpu().numpy()
        
            if np_boxes.size == 0:
                no_detection_count += 1
                if no_detection_count > 3:
                    self.playSound()
            else:
                self.target_found = True
                return 'target_found'

            # sleep for 0.1s then detect again
            self.node.get_clock().sleep_for(Duration(seconds=0.1))