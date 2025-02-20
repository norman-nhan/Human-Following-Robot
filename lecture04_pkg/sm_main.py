#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

# Import modules (ROS2 related)
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


# Import modules (YASMIN related)
# https://github.com/uleroboticsgroup/yasmin.git
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

# Import modules (Custom: Each state)
from .states import following

class StateMachineNode(Node):
    """StateMachineNode class (inherits from Node class)
    Node class that executes the state machine
    """

    def __init__(self):
        """Class initialization method"""
        super().__init__("sm_main")

        self.get_logger().info("\033[43m\033[30m\033[1m<< PLEASE ENTER TO START >>\033[0m")
        input()
        self.get_logger().info("Task Start!!")

        self.vel_pub = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

        # Create an instance of the StateMachine class
        sm = StateMachine(outcomes=["EXIT"])

        # Add InitializationState to the state machine
        sm.add_state(
            name="Initialization",
            state=???(node=self),
            transitions={???: "Following"},
        )

        # Add FollowingState to the state machine
        sm.add_state(
            name="Following",
            state=following.FollowingState(node=self),
            transitions={"QRFound": ???, "TargetLost":???},
        )

        # Add ???State to the state machine
        # Searching for the newly lost target       
        sm.add_state(
            name=???,
            state=???(node=self),
            transitions={"TargetFound": "Following"},
        )

        # Add ???State to the state machine
        # Using QR Code to reach the goal       
        sm.add_state(
            name=???,
            state=???(node=self),
            transitions={???: "EXIT"},
        )

        # Publish state machine information to Yasmin Viewer
        YasminViewerPub(fsm_name="SM_MAIN", fsm=sm)

        # Execute the state machine
        outcome = sm()
        self.get_logger().info("State Machine finished with outcome: " + outcome)


def shutdown(node: Node):
    """Shutdown function
    Stop TurtleBot3 when terminating

    Args:
        node (Node): Node object
    """
    node.get_logger().info("Follow State Cleanup!!")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    pub.publish(Twist())
    node.get_clock().sleep_for(Duration(nanoseconds=100))
    node.destroy_publisher(pub)


def main(args=None):
    """Main function"""
    # Initialize ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of StateMachineNode class
    try:
        node = StateMachineNode()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
