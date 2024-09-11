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

from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

import tf_transformations
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

import cv2
import cv2.aruco as aruco
import numpy as np
import time


class Go2GoalState(State):
    """Read AruCo marker then go to goal pose."""
    def __init__(self, node: Node):
        super().__init__(outcomes=['succeed', 'failed'])

        # Initialize instance variables
        self._goal_handle: ClientGoalHandle = None
        self._result_future: NavigateToPose_GetResult_Response = None
        self._feedback: NavigateToPose_Feedback = None
        self._status: int = None

        self.node = node

        self.nav_to_pose_client = ActionClient(
            node=self.node, action_type=NavigateToPose, action_name="/navigate_to_pose"
        )

        # Initialize TF to transform pose in camera's coordinate frame to pose in robot's base_link frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Subscribe to camera's info topic to get camera info (camera matrix, lens distortion coefficient vector)
        self.camera_info_received = False
        self.camera_info_sub = self.node.create_subscription(
            msg_type=CameraInfo, topic='/camera_info', callback=self.camera_info_callback, qos_profile=10
        )
        
        self.target_id = 0 # Go to desired pose if read this id
        self.marker_length = 0.15  # meters
        self.desired_offset = np.array([0.0, 0.0, -0.5])  # 50cm behind marker

        # Subscribe to camera's image topic
        self.bridge = CvBridge()
        self.cv_image = None
        self.image_sub = self.node.create_subscription(
            msg_type=Image, topic="image_raw", callback=self.image_callback, qos_profile=10
        )
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

    def image_callback(self, msg: Image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.node.get_logger().info(e)

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_mat = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.camera_info_received = True
            self.node.get_logger().info(f"Camera Info received: {self.camera_mat, self.dist_coeffs}")

    def get_aruco_detection_results(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, channels = detector.detectMarkers(gray)

        return corners, ids, channels

    def detect_and_navigate(self):
        while self.cv_image is None and rclpy.ok():
            self.node.get_logger().info('Waiting for camera image...')
            time.sleep(0.1)
    
        corners, ids, _ = self.get_aruco_detection_results(self.cv_image)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_mat, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != self.target_id:
                    continue  # Skip other markers

                rvec, tvec = rvecs[i][0], tvecs[i][0]
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])

                desired_pos = tvec + rot_mat @ self.desired_offset

                # Draw for debug
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.drawFrameAxes(frame, self.camera_mat, self.dist_coeffs, rvec, tvec, 0.1)
                cv2.putText(frame, f"ID: {marker_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                self.node.get_logger().info(f"Detected marker {marker_id}, navigating to offset pose")

                try:
                    # Create PoseStamped in camera frame
                    pose_cam = PoseStamped()
                    pose_cam.header.stamp = self.node.get_clock().now().to_msg()
                    pose_cam.header.frame_id = "camera_link"  # Replace with actual frame, e.g., "camera_color_optical_frame"
                    pose_cam.pose.position.x = desired_pos[0]
                    pose_cam.pose.position.y = desired_pos[1]
                    pose_cam.pose.position.z = desired_pos[2]
                    quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
                    pose_cam.pose.orientation.x = quat[0]
                    pose_cam.pose.orientation.y = quat[1]
                    pose_cam.pose.orientation.z = quat[2]
                    pose_cam.pose.orientation.w = quat[3]

                    # Transform to map frame
                    # make sure that there is camera's frame in robot's TF tree! Otherwise, this won't work.
                    transform = self.tf_buffer.lookup_transform(
                        "map",
                        pose_cam.header.frame_id,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    pose_map = do_transform_pose(pose_cam, transform)

                    self.node.get_logger().info(f"Transformed pose: x={pose_map.pose.position.x:.2f}, y={pose_map.pose.position.y:.2f}, yaw={yaw:.2f}")
                    self.goToPose(pose_map.pose.position.x, pose_map.pose.position.y, yaw)

                except Exception as e:
                    self.node.get_logger().error(f"TF transform failed: {e}")
                    self.node.get_logger().error(f"Cannot go to goal pose.")

                time.sleep(1)
                break  # Navigate once per detection

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
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected!")
            return

        self.node.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_result_future)
        result = get_result_future.result().result

        if get_result_future.result().status == 4:  # CANCELED
            self.node.get_logger().warn("Navigation canceled.")
        elif get_result_future.result().status != 3:  # SUCCEEDED
            self.node.get_logger().error("Navigation failed!")
        else:
            self.node.get_logger().info("Navigation succeeded!")

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
        self.node.get_logger().info("Go2GoalState")

        if not rclpy.ok:
            return "failed"
        
        self.detect_and_navigate()

        # Wait until navigation is complete
        while not self.isNavComplete():
            # Get feedback message
            feedback = self.getFeedback()
            # Cancel if navigation takes more than 30 seconds
            if feedback.navigation_time > 30:
                self.cancelNav()  # Cancel the ongoing navigation

        # Get and display navigation result
        result = self.getResult()
        match result:
            case GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info("Navigation succeeded!")
                return "succeed"
            case GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("Navigation was canceled!")
                return "failed"
            case GoalStatus.STATUS_ABORTED:
                self.node.get_logger().error("Navigation failed!")
                return "failed"
            case _:
                self.node.get_logger().error("Unknown error!")
                return "failed"

        return "succeed"