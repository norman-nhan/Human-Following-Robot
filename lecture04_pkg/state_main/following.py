#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

# Import external modules
import numpy as np
import cv2
import cv2.aruco as aruco
from ultralytics import YOLO
# Import modules (ROS2 related)
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# Import modules (YASMIN related)
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import State, Blackboard

class FollowingState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=["qr_found", "target_lost", "loop"])

        # init ros node
        self.node = node

        # init cv image
        self.bridge = CvBridge()
        self.cv_image = None
        self.rendered_image = None

        # init publishers and subscribers
        self.image_sub = self.node.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.image_pub = self.node.create_publisher(Image, 'yolo_results', 10)
        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.vel_sub = self.node.create_subscription(Twist, "cmd_vel", self.vel_callback, 10)

        # init robot's velocity command
        self.cmd_vel = Twist()
        self.max_linear_vel = 0.22
        self.max_angular_vel = 2.84
        self.speed = 0.05

        # init model
        self.model = YOLO('/home/ros2/ros2_lecture_ws/src/7_lectures/lecture04_pkg/lecture04_pkg/models/best_ali.pt')
        self.no_detection_count = 0

        # init aruco detection
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

    def vel_callback(self, msg: Twist):
        """Get robot linear, angular velocity"""
        self.node.get_logger().debug('Receiving cmd vel')
        self.cmd_vel = msg

    def image_callback(self, msg: Image): 
        """Get a cv image."""
        self.node.get_logger().debug('Receiving image')
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # pass cv image to self.cv_image
        except CvBridgeError as e:
            self.node.get_logger().info(str(e))

    def pub_yolo_image(self):
        """Publish image with yolo bounding boxes inside."""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.rendered_image, 'bgr8')
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            self.node.get_logger().error(f'Error converting image: {e}')

    def get_bounding_boxes(self):
        """Get all bounding boxes from the first result and draw it.
        
        Returns
        -------
        boxes: Tensor
            A tensor contains a list of boxes. 

            Each box contains x_center, y_center, width, height.
        """
        assert self.cv_image is not None, "No cv image received."
        results = self.model(self.cv_image)
        self.rendered_image = results[0].plot() # Draw results

        if len(results[0].boxes.xywh) == 0: # No boxes
            return None

        boxes = results[0].boxes.xywh
        return boxes

    def get_follower_posi_and_dist(self, largest_box):
        """Get the follower's position and distance.
        
        Returns
        -------
        posi: str
            position of follower in image: 'left', 'center', 'right'.
        dist: str
            distance to follower: 'near', 'far', 'median'.

        Parameters
        ----------
        largest_box: np.array
            An array that contains [x_center, y_center, width, height] corresponds to bounding box.
        """
        assert largest_box is not None, "No largest_box received."
        image_heigth, image_width = self.cv_image.shape[:2] # cv_image.shape contains a tuple of (height, width, channel)
        box_x_center, _, _, box_height = largest_box

        if box_x_center < image_width / 3.0:
            posi = 'left'
        elif box_x_center > (2.0/3.0) * image_width:
            posi = 'right'
        else:
            posi = 'center'

        if box_height > 0.6 * image_heigth:
            dist = "near"
        elif box_height < 0.25 * image_heigth:
            dist = "far"
        else:
            dist = "medium"

        # draw onto result
        cv2.putText(self.rendered_image, f'Posi: {posi}, Dist: {dist}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        self.node.get_logger().info(f'posi: {posi}, box_x_center: {box_x_center}, image_height: {image_heigth}')
        self.node.get_logger().info(f'dist: {dist}, box_height: {box_height}')
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
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel.linear.x - self.speed)
            case 'medium':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel.linear.x)
            case 'far':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel.linear.x + self.speed)

        match posi: # follower position in image
            case 'left':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z + self.speed)
            case 'center':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z)
            case 'right':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z - self.speed)
        return cmd_vel

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing Following State")
        while rclpy.ok():
            rclpy.spin_once(self.node)
            ## Handling if cv_image is None
            if self.cv_image is None:
                self.node.get_logger().info(f'{self.__class__.__name__}: Waiting for image ...')
                continue

            ## AruCo marker Dectection ##
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
            _, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                self.node.get_logger().info(f'detected_id: {str(ids[0])}')
                self.vel_pub.publish(Twist()) # send stopping signal to robot
                return 'qr_found'

            ## (YOLO) Band bounding box Detection ##
            boxes = self.get_bounding_boxes()

            # drawing image
            if self.rendered_image is None:
                self.rendered_image = self.cv_image.copy()

            if boxes is None:  # No boxes
                self.no_detection_count += 1
                self.pub_yolo_image()
                if self.no_detection_count > 20:
                    self.vel_pub.publish(Twist())
                    return 'target_lost'
                else:
                    self.node.get_clock().sleep_for(Duration(seconds=0.5))
                    continue
            else:
                self.no_detection_count = 0 # reset self.no_detection_count when detected
            largest_box = max(boxes.cpu().numpy(), key=lambda x: x[2] * x[3]) # get the largest boudning box
            posi, dist = self.get_follower_posi_and_dist(largest_box)
            cmd_vel = self.get_dynamic_vel(posi, dist)
            self.pub_yolo_image()
            self.node.get_logger().info(f'Sending cmd_vel: {str(cmd_vel)}')
            self.vel_pub.publish(cmd_vel)