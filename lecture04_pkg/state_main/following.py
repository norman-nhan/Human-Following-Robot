#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

# Import external modules
import numpy as np
import cv2
import cv2.aruco as aruco
from ultralytics import YOLO
import threading
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
        super().__init__(outcomes=["qr_found", "target_lost"])

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
        self.max_linear_vel = 0.22 / 2.0
        self.max_angular_vel = 2.84 / 2.0
        self.speed = 0.1

        # init model
        self.model = YOLO('/home/ros2/ros2_lecture_ws/src/7_lectures/lecture04_pkg/lecture04_pkg/models/best_krisna2.pt')
        self.frame_count = 0 # skip using yolo too frequently 

        # init aruco detection
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        # init threading
        self.thread_running = True
        self.processing_thread = threading.Thread(target=self.image_processing_loop)
        self.processing_thread.start()

        self.yolo_result = None
        self.qr_found_flag = False

    def cleanup(self):
        self.thread_running = False
        self.processing_thread.join()

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

    def get_follower_posi_and_dist(self, largest_box):
        """Get the follower's position and distance.
        
        Returns
        -------
        posi: str
            position of follower in image: 'left', 'center', 'right'.
        dist: str
            distance to follower: 'near', 'far', 'medium'.

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

        who_model_dict = {
            'ali': {'near': 0.4, 'far': 0.1},
            'krisna': {'near': 0.95, 'far': 0.80}
        }
        model_dict = who_model_dict['krisna']
        if box_height > model_dict['near'] * image_heigth:
            dist = "near"
        elif box_height < model_dict['far'] * image_heigth:
            dist = "far"
        else:
            dist = "medium"

        # draw onto result
        cv2.putText(self.rendered_image, f'Posi: {posi}, Dist: {dist}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        self.node.get_logger().info(f'posi: {posi}, box_x_center: {box_x_center}, image_width: {image_width}')
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
                if self.cmd_vel.linear.x == 0: # check if there was no velocity published
                    cmd_vel.linear.x = 0.1
            case 'far':
                cmd_vel.linear.x = min(self.max_linear_vel, self.cmd_vel.linear.x + self.speed)

        match posi: # follower position in image
            case 'left':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z + self.speed)
            case 'center':
                cmd_vel.angular.z = 0.0
            case 'right':
                cmd_vel.angular.z = min(self.max_angular_vel, self.cmd_vel.angular.z - self.speed)
        return cmd_vel

    def get_krisna_model_result(self, results):
        boxes_data = results[0].boxes.data
        boxes_xywh = results[0].boxes.xywh 
        mask = boxes_data[:, 5] == 0
        boxes = boxes_xywh[mask]

        return boxes

    def image_processing_loop(self):
        no_detection_count = 0
        while self.thread_running:
            if self.cv_image is None:
                continue

            # ArUco detection
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
            _, ids, _ = detector.detectMarkers(gray)
            if ids is not None and ids[0] == 0:
                self.node.get_logger().info(f'detected_id: {str(ids[0])}')
                self.qr_found_flag = True
                break  # Exit thread

            # YOLO detection
            results = self.model(self.cv_image, conf=0.6)
            boxes = self.get_krisna_model_result(results)
            # self.node.get_logger().info(f'{str(boxes)}')
            np_boxes = boxes.cpu().numpy()
            # boxes = results[0].boxes.xywh # ali's model

            self.rendered_image = results[0].plot()

            # if len(results[0].boxes.xywh) == 0: # there is no band detected
            if np_boxes.size == 0:
                no_detection_count += 1
                self.vel_pub.publish(Twist())
                if no_detection_count > 200:
                    self.pub_yolo_image()
                    self.yolo_result = 'target_lost'
                    break
                self.pub_yolo_image()
                continue
            else:
                no_detection_count = 0

            largest_box = max(boxes.cpu().numpy(), key=lambda x: x[2] * x[3])
            self.node.get_logger().info(f'-----LARGEST BOX------: {str(largest_box)}')
            posi, dist = self.get_follower_posi_and_dist(largest_box)
            self.node.get_logger().info(f'POSI: {posi}, DIST: {dist}')
            self.pub_yolo_image()
            cmd_vel = self.get_dynamic_vel(posi, dist)
            self.vel_pub.publish(cmd_vel)

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing Following State")
        self.qr_found_flag = False  # Reset
        self.yolo_result = None     # Reset

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.qr_found_flag:
                self.vel_pub.publish(Twist())  # Stop robot
                return 'qr_found'

            if self.yolo_result == 'target_lost':
                self.vel_pub.publish(Twist())  # Stop robot
                return 'target_lost'

            # Sleep a little to reduce CPU usage
            self.node.get_clock().sleep_for(Duration(seconds=5/1000.0))

