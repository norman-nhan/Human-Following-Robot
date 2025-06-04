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
        super().__init__(outcomes=['succeed', 'failed'])

        self._goal_handle: ClientGoalHandle = None
        self._result_future: NavigateToPose_GetResult_Response = None
        self._feedback: NavigateToPose_Feedback = None
        self._status: int = None

        self.node = node

        self.nav_to_pose_client = ActionClient(
            node=self.node, action_type=NavigateToPose, action_name="/navigate_to_pose"
        )

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

        self.target_id = 0 # Go to desired pose if read this id
        self.marker_length = 0.15  # meters
        self.goal_offset = -0.5
        self.is_lock = False
        # Subscribe to camera's image topic
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_sub = self.node.create_subscription(Image, "image_raw", self.image_callback, 10)
        self.image_pub = self.node.create_publisher(Image, 'aruco_image', 10)
        self.marker_length = 9.4 / 100.0 # meters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        self.aruco_params = aruco.DetectorParameters()
        
        # Tuning parameters
        self.aruco_params.adaptiveThreshWinSizeMin = 5
        self.aruco_params.adaptiveThreshWinSizeMax = 21
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.adaptiveThreshConstant = 7

        # self.aruco_params.minMarkerPerimeterRate = 0.03
        # self.aruco_params.maxMarkerPerimeterRate = 4.0
        # self.aruco_params.polygonalApproxAccuracyRate = 0.03
        # self.aruco_params.minCornerDistanceRate = 0.05

    def move_to_goal(self, T_goal_robot):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()

        pose_msg.pose.position.x = T_goal_robot[0, 3]
        pose_msg.pose.position.y = T_goal_robot[1, 3]
        pose_msg.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_matrix(T_goal_robot)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        # Wait for the action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("NavigateToPose action server not available!")
            return False        
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal=goal_msg,  # Goal message
            feedback_callback=self._feedbackcallback,  # Callback method
        )
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)
        self._goal_handle = send_goal_future.result()

        if not self._goal_handle.accepted:
            self.node.get_logger().error("Goal rejected!")
            return

        # Set up to get the result asynchronously
        self._result_future = self._goal_handle.get_result_async()
        self.node.get_logger().info("Goal accepted, waiting for result...")

        result = self._result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("Navigation succeeded!")
            return True
        else:
            self.node.get_logger().error(f"Navigation failed with status code: {result.status}")
            return False

    def detect_aruco(self):
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

            # Estimate pose of each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_mat, self.dist_coeffs)

            for i in range(len(ids)):
                # Draw axis for each marker
                cv2.drawFrameAxes(frame, self.camera_mat, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length * 0.5)

            for i, marker_id in enumerate(ids):
                if marker_id != self.target_id:
                    continue

                rvec, tvec = rvecs[i][0], tvecs[i][0]
                R_ct, _ = cv2.Rodrigues(rvec)
                T_ct = np.eye(4)
                T_ct[:3, :3] = R_ct
                T_ct[:3, 3] = tvec.flatten()

                T_goal_marker = np.eye(4)
                T_goal_marker[0, 3] = self.goal_offset

                T_goal_robot = T_ct @ T_goal_marker
                # self.move_to_goal(T_goal_robot)
                self.node.get_logger().info(f'T_goal_robot: {str(T_goal_robot)}')
                self.is_lock = True
                break
        else:
            self.node.get_logger().info('No ArUco markers detected.')

        # Convert the (possibly unannotated) image to a ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.node.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_link"  # Change as appropriate
        self.image_pub.publish(image_msg)

    def image_callback(self, msg: Image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(e)

    def goToPose(self, x: float, y: float, yaw: float):
        """Method to navigate to a specified destination

        Args:
            x (float): X coordinate of the destination [m]
            y (float): Y coordinate of the destination [m]
            yaw (float): Orientation at the destination (Yaw angle) [rad]

        Returns:
            bool: Response from NavigateToPose server. True (accepted), False (rejected)
        """

        self.node.get_logger().debug("Waiting for NavigateToPose server...")

        # Wait until connection with NavigationPose server is established
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn("Waiting for 'NavigateToPose' action server...")

        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, float(yaw))
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.node.get_logger().info(f"Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal=goal_msg,  # Goal message
            feedback_callback=self._feedbackcallback,  # Callback method
        )
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)
        self._goal_handle = send_goal_future.result()

        if not self._goal_handle.accepted:
            self.node.get_logger().error("Goal rejected!")
            return

        # Set up to get the result asynchronously
        self._result_future = self._goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """Method to cancel the ongoing navigation"""
        self.node.get_logger().info("Canceling current task.")
        if self._result_future:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, future)

    def isNavComplete(self) -> bool:
        """Method to check the completion status of navigation

        Returns:
            bool: Returns the status. True (canceled, completed), False (timeout, in progress, incomplete)
        """
        if not self._result_future:
            return True
        rclpy.spin_until_future_complete(self.node, self._result_future, timeout_sec=0.10)
        if self._result_future.result():
            self._status = self._result_future.result().status
            if self._status != GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().debug(f"Task with failed with status code: {self._status}")
                return True
        else:
            return False
        self.node.get_logger().debug(f"Navigation succeeded!")
        return True

    def getResult(self) -> int:
        """Method to get the result message of the pending action

        Returns:
            int: Returns the status code.
        """
        return self._status

    def getFeedback(self) -> NavigateToPose_Feedback:
        """Method to get the feedback message of the pending action

        Returns:
            NavigateToPose_Feedback: Returns a NavigateToPose_Feedback object
        """
        return self._feedback

    def _feedbackcallback(self, msg: NavigateToPose_FeedbackMessage):
        """Callback method called when feedback is received from the server

        Args:
            feedback_msg (NavigateToPose_FeedbackMessage): Feedback message of type nav2_msgs/action/NavigateToPose
        """
        # Get feedback
        self.node.get_logger().debug("Received action feedback message")
        self._feedback = msg.feedback

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing Go2Goal State")

        while rclpy.ok():
            if self.is_lock:
                # We have detected the marker and computed T_goal_robot
                success = self.move_to_goal(self.T_goal_robot)
                if success:
                    return "succeed"
                else:
                    return "failed"
            
            self.detect_aruco()
            rclpy.spin_once(self.node)

        return "failed"  # if rclpy is shutdown before detection/navigation