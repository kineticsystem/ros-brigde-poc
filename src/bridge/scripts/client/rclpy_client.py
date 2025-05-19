#!/usr/bin/env python3

"""
Simple action client for testing the TestTimer action server.
This client can be used to test the action server functionality.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from bridge_msgs.action import BridgeAction


class TestActionClient(Node):
    def __init__(self):
        super().__init__("test_action_client")
        self._action_client = ActionClient(self, BridgeAction, "test_action")

    def send_goal(self, seconds):
        goal_msg = BridgeAction.Goal()
        goal_msg.number_of_seconds = seconds

        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending goal: {seconds} seconds")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.message}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Received feedback: {feedback.seconds_elapsed} seconds elapsed"
        )


def main(args=None):
    rclpy.init(args=args)

    action_client = TestActionClient()

    # Send a goal for 10 seconds
    action_client.send_goal(10)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
