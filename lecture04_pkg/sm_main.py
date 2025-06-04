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
from .state_main import (
    init_state,
    following, 
    go2goal, 
#    screaming
)
class StateMachineNode(Node):
    def __init__(self):
        super().__init__("sm_main")

        # self.get_logger().info("\033[43m\033[30m\033[1m<< PLEASE ENTER TO START >>\033[0m")
        # input() # if you use ros2 launch to run this file, the input will be detached with terminal. So that is why when you press enter, nothing happens. 
        self.get_logger().info("\033[43m\033[30m\033[1m<< TASKS START >>\033[0m")

        self.vel_pub = self.create_publisher(msg_type=Twist, topic="cmd_vel", qos_profile=10)

        # Create an instance of the StateMachine class
        sm = StateMachine(outcomes=["EXIT"])

        ## Adding states
        sm.add_state(
            name="InitState",
            state=init_state.InitState(node=self),
            transitions={
                'succeed': "FollowingState",
                # 'succeed': 'Go2GoalState'
            },
        )
        sm.add_state(
            name="FollowingState",
            state=following.FollowingState(node=self),
            transitions={
                'loop': 'FollowingState',
                "qr_found": 'Go2GoalState',
            },
        )
        sm.add_state(
            name='Go2GoalState',
            state=go2goal.Go2GoalState(node=self),
            transitions={
                'succeed': 'EXIT',  
                'failed': 'Go2GoalState',
                'qr_lost': 'FollowingState'
            },
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
    rclpy.init(args=args)

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
