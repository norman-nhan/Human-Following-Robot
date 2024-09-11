#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from yasmin import State, Blackboard

class InitState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['succeed'])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info('Start to following!')
        return 'succeed'