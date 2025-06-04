#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

"""
File: navigation.py
Author: Tomoaki Fujino（Kyushu Institute of Technology, Hibikino-Musashi@Home）
"""

# Import modules (ROS2 related)
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
import tf_transformations

# Import modules (YASMIN related)
# https://github.com/uleroboticsgroup/yasmin.git
import rclpy.task
from yasmin import State
from yasmin import Blackboard


class NavigationState(State):
    """NavigationState class (inherits from State class)
    Uses navigation to return to the initial position
    """

    def __init__(self, node: Node):
        """Class initialization method"""
        # Override the constructor of the inherited State class
        # The outcomes argument specifies the possible results to return when the state completes
        super().__init__(outcomes=["succeed", "failed"])

        # Initialize instance variables
        self._goal_handle: ClientGoalHandle = None
        self._result_future: NavigateToPose_GetResult_Response = None
        self._feedback: NavigateToPose_Feedback = None
        self._status: int = None

        self.node = node

        self.nav_to_pose_client = ActionClient(
            node=self.node, action_type=NavigateToPose, action_name="/navigate_to_pose"
        )

    def goToPose(self, x: float, y: float, yaw: float) -> bool:
        """Method to navigate to a specified destination

        Args:
            x (float): X coordinate of the destination [m]
            y (float): Y coordinate of the destination [m]
            yaw (float): Orientation at the destination (Yaw angle) [rad]

        Returns:
            bool: Response from NavigateToPose server. True (accepted), False (rejected)
        """

        self.node.get_logger().debug("Waiting for 'NavigateToPose' action server")

        # Wait until connection with NavigateToPose server is established
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info("'NavigateToPose' action server not available, waiting...")

        # Create a PoseStamped message and set the destination coordinates and orientation
        pose = PoseStamped()

        ## Set the destination coordinates
        pose.header.stamp = self.node.get_clock().now().to_msg()  # Set current time in the header
        pose.header.frame_id = "map"  # Set frame ID to 'map'
        pose.pose.position.x = x  # X coordinate of the destination [m]
        pose.pose.position.y = y  # Y coordinate of the destination [m]
        pose.pose.position.z = 0.0  # Z coordinate of the destination [m] # 0.0 for 2D plane

        ## Set the orientation at the destination
        ### Convert Euler angles to quaternion
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        pose.pose.orientation.w = quat[0]

        # Create a Goal message for NavigateToPose action
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Display log
        self.node.get_logger().info(f"Navigating to goal: (x, y, yaw) = ({x}, {y}, {yaw})")

        # Send the Goal message to the action server asynchronously and register a callback method to be called when feedback messages are received
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal=goal_msg,  # Goal message
            feedback_callback=self._feedbackcallback,  # Callback method
        )
        # Wait until the Goal message is sent
        rclpy.spin_until_future_complete(self, send_goal_future)

        # Get the result of sending the Goal message
        self._goal_handle = send_goal_future.result()

        # Check the acceptance result from the NavigateToPose server
        if not self._goal_handle.accepted:
            self.node.get_logger().error(f"Goal to (x, y, yaw) = ({x}, {y}, {yaw}) was rejected!")
            return False

        # Set up to get the result asynchronously
        self._result_future = self._goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """Method to cancel the ongoing navigation"""
        self.node.get_logger().info("Canceling current task.")
        if self._result_future:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)

    def isNavComplete(self) -> bool:
        """Method to check the completion status of navigation

        Returns:
            bool: Returns the status. True (canceled, completed), False (timeout, in progress, incomplete)
        """
        if not self._result_future:
            return True
        rclpy.spin_until_future_complete(self, self._result_future, timeout_sec=0.10)
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
        """
        Execution method for Navigation state

        Args:
            blackboard (CustomBlackboard): CustomBlackboard object

        Returns:
            str: outcomes string
        """
        self.node.get_logger().info("Navigation")
        if not rclpy.ok:
            return "failed"

        # Start navigation (move to initial position)
        self.goToPose(x=0.0, y=0.0, yaw=0.0)

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
                self.node.get_logger().info("Nvigation succeeded!")
                return "succeed"
            case GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("Nvigation was canceled!")
                return "failed"
            case GoalStatus.STATUS_ABORTED:
                self.node.get_logger().error("Nvigation failed!")
                return "failed"
            case _:
                self.node.get_logger().error("Unknown error!")
                return "failed"

        return "succeed"
