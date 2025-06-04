#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import (
    NavigateToPose_GetResult_Response,
    NavigateToPose_Feedback,
    NavigateToPose_FeedbackMessage,
)

from yasmin import State, Blackboard
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf_transformations
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

import cv2
import cv2.aruco as aruco
import numpy as np

class Go2GoalState(State):
    """Read AruCo marker then go to goal pose."""
    def __init__(self, node: Node):
        super().__init__(outcomes=['succeed', 'failed', 'qr_lost'])

        self.node = node

        self.vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        # Matrices needed for rigid transformation
        self.camera_mat = np.array([
            [552.3676403778572, 0.0, 271.5405270219728],
            [0.0, 549.8865688457811, 248.41840056895452],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([
            -0.1545960462219134,
            0.008160371579626012,
            0.002019177429942932,
            0.002507781319119664,
            0.1994557667554336
        ])

        ## AruCo Dectection Logic
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_sub = self.node.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.image_pub = self.node.create_publisher(Image, 'aruco_image', 10)
        self.marker_length = 9.4 / 100.0 # meters
        self.target_id = 0 # Go to desired pose if read this id
        self.goal_offset = 0.1
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        self.aruco_params = aruco.DetectorParameters()
        self.qr_lost = False
        # Tuning parameters
        # self.aruco_params.adaptiveThreshWinSizeMin = 5
        # self.aruco_params.adaptiveThreshWinSizeMax = 21
        # self.aruco_params.adaptiveThreshWinSizeStep = 4
        # self.aruco_params.adaptiveThreshConstant = 7

        # self.aruco_params.minMarkerPerimeterRate = 0.03
        # self.aruco_params.maxMarkerPerimeterRate = 4.0
        # self.aruco_params.polygonalApproxAccuracyRate = 0.03
        # self.aruco_params.minCornerDistanceRate = 0.05

    def move_to_marker(self, forward_dist, lateral_offset):
        self.node.get_logger().info('Executing move_to_marker() method.')
        goal_dist = 0.05
        error_x = forward_dist - goal_dist
        error_y = lateral_offset
        self.node.get_logger().info(f'err_x: {error_x}')
        self.node.get_logger().info(f'err_y: {error_y}')
        
        vx = 0.2 * error_x
        vy = 0.0 # ignored unless holonomic robot
        wz = 1.0 * error_y

        if abs(error_x) < 0.05 and abs(error_y) < 0.05:
            vx = 0.0
            wz = 0.0
            self.vel_pub.publish(Twist())

        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.node.get_logger().info(f'Publishing vel: {str(msg)}')
        self.vel_pub.publish(msg)

    def detect_aruco(self) -> bool:
        while self.cv_image is None:
            rclpy.spin_once(self.node)
            continue

        # Convert to grayscale for detection
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)

        # Copy the original image to draw markers on
        frame = self.cv_image.copy()
        if ids is not None and len(ids.flatten()) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != self.target_id:
                    continue
                corner = corners[i]
                # Estimate pose of each marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, self.marker_length, self.camera_mat, self.dist_coeffs)

                cv2.drawFrameAxes(frame, self.camera_mat, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)

                tvec = tvec.flatten()

                x = tvec[2]
                y = -tvec[0]
                self.move_to_marker(x, y)
                break
        else:
            self.node.get_logger().info('No ArUco markers detected.')
            self.vel_pub.publish(Twist())
            return False
        
        # Convert the (possibly unannotated) image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.node.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_link"  # Change as appropriate
        self.image_pub.publish(image_msg)
        return True
    
    def image_callback(self, msg: Image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(e)

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing Go2Goal State")
        no_qr_code_found = 0
        while rclpy.ok():
            is_detected= self.detect_aruco()
            if not is_detected:
                no_qr_code_found += 1
                if no_qr_code_found > 3:
                    return 'qr_lost'
            rclpy.spin_once(self.node)
        return "succeed"
