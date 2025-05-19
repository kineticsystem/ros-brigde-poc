#!/usr/bin/env python3

"""
ROS2 Action Server for testing ROS Bridge connectivity.
This server accepts a timer goal and runs for the specified duration,
providing feedback and handling cancellation using ROS2 timers and Duration.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
import threading

from bridge_msgs.action import BridgeAction


class TestActionServer(Node):
    def __init__(self):
        super().__init__("test_action_server")

        # Create callback group for action server
        self._action_callback_group = ReentrantCallbackGroup()

        # Create action server
        self._action_server = ActionServer(
            self,
            BridgeAction,
            "test_action",
            self.execute_callback,
            callback_group=self._action_callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Timer and goal tracking
        self._check_timer = None
        self._current_goal_handle = None
        self._start_time = None
        self._target_duration = None
        self._last_feedback_second = -1
        self._execution_complete = None  # Event to signal completion

        self.get_logger().info("Test Action Server is ready")

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f"Received goal request for {goal_request.number_of_seconds} seconds"
        )

        # Validate the goal
        if goal_request.number_of_seconds <= 0:
            self.get_logger().warn("Rejecting goal: number_of_seconds must be positive")
            return GoalResponse.REJECT

        if goal_request.number_of_seconds > 300:  # 5 minutes max
            self.get_logger().warn("Rejecting goal: number_of_seconds must be <= 300")
            return GoalResponse.REJECT

        # Cancel any existing timer if there's an active goal
        if self._check_timer is not None:
            self._check_timer.cancel()
            self._check_timer = None

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the action using ROS2 timers and Duration."""
        self.get_logger().info("Executing goal...")

        # Store goal information
        self._current_goal_handle = goal_handle
        self._target_duration = Duration(seconds=goal_handle.request.number_of_seconds)
        self._start_time = self.get_clock().now()
        self._last_feedback_second = -1
        self._execution_complete = threading.Event()

        # Create a timer that fires every 100ms for responsive cancellation
        # but only sends feedback every second
        self._check_timer = self.create_timer(
            0.1,  # 100ms for responsive cancellation
            self._check_timer_callback,
            callback_group=self._action_callback_group,
        )

        # Send initial feedback immediately
        feedback_msg = BridgeAction.Feedback()
        feedback_msg.seconds_elapsed = 0
        self._current_goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(
            f"Seconds elapsed: 0/{goal_handle.request.number_of_seconds}"
        )

        # Wait for execution to complete (either success or cancellation)
        self._execution_complete.wait()

        # Return the final result
        if goal_handle.is_cancel_requested:
            # Calculate elapsed time at cancellation
            current_time = self.get_clock().now()
            elapsed_duration = current_time - self._start_time
            elapsed_seconds = elapsed_duration.nanoseconds / 1e9

            result = BridgeAction.Result()
            result.message = f"Action canceled after {elapsed_seconds:.2f} seconds"
            return result
        else:
            # Calculate final elapsed time
            current_time = self.get_clock().now()
            elapsed_duration = current_time - self._start_time
            elapsed_seconds = elapsed_duration.nanoseconds / 1e9
            target_seconds = self._target_duration.nanoseconds / 1e9

            result = BridgeAction.Result()
            result.message = f"Action completed successfully. Target: {target_seconds}s, Actual: {elapsed_seconds:.2f}s"
            return result

    def _check_timer_callback(self):
        """Timer callback to check progress, send feedback, and handle completion/cancellation."""
        if self._current_goal_handle is None:
            return

        # Check if goal is canceled
        if self._current_goal_handle.is_cancel_requested:
            self._handle_cancellation()
            return

        # Calculate elapsed time using ROS2 Duration
        current_time = self.get_clock().now()
        elapsed_duration = current_time - self._start_time
        elapsed_seconds = int(elapsed_duration.nanoseconds / 1e9)

        # Send feedback only when a new second has elapsed
        if elapsed_seconds != self._last_feedback_second:
            self._last_feedback_second = elapsed_seconds

            # Send feedback
            feedback_msg = BridgeAction.Feedback()
            feedback_msg.seconds_elapsed = elapsed_seconds
            self._current_goal_handle.publish_feedback(feedback_msg)

            target_seconds = int(self._target_duration.nanoseconds / 1e9)
            self.get_logger().info(
                f"Seconds elapsed: {elapsed_seconds}/{target_seconds}"
            )

        # Check if we've reached or exceeded the target duration
        if elapsed_duration >= self._target_duration:
            self._handle_success()

    def _handle_cancellation(self):
        """Handle goal cancellation."""
        if self._current_goal_handle is None:
            return

        # Calculate elapsed time at cancellation
        current_time = self.get_clock().now()
        elapsed_duration = current_time - self._start_time
        elapsed_seconds = elapsed_duration.nanoseconds / 1e9

        self.get_logger().info(f"Goal canceled after {elapsed_seconds:.2f} seconds")

        # Stop the timer
        if self._check_timer is not None:
            self._check_timer.cancel()
            self._check_timer = None

        # Mark as canceled
        self._current_goal_handle.canceled()

        # Signal that execution is complete
        if self._execution_complete is not None:
            self._execution_complete.set()

        self._current_goal_handle = None

    def _handle_success(self):
        """Handle successful goal completion."""
        if self._current_goal_handle is None:
            return

        # Calculate final elapsed time
        current_time = self.get_clock().now()
        elapsed_duration = current_time - self._start_time
        elapsed_seconds = elapsed_duration.nanoseconds / 1e9
        target_seconds = self._target_duration.nanoseconds / 1e9

        self.get_logger().info(
            f"Goal succeeded! Completed in {elapsed_seconds:.2f} seconds"
        )

        # Stop the timer
        if self._check_timer is not None:
            self._check_timer.cancel()
            self._check_timer = None

        # Mark as succeeded
        self._current_goal_handle.succeed()

        # Signal that execution is complete
        if self._execution_complete is not None:
            self._execution_complete.set()

        self._current_goal_handle = None

    def _get_elapsed_seconds_float(self):
        """Get elapsed seconds as a float for more precise measurements."""
        if self._start_time is None:
            return 0.0

        current_time = self.get_clock().now()
        elapsed_duration = current_time - self._start_time
        return elapsed_duration.nanoseconds / 1e9

    def _get_target_seconds_float(self):
        """Get target seconds as a float."""
        if self._target_duration is None:
            return 0.0
        return self._target_duration.nanoseconds / 1e9

    def destroy_node(self):
        """Clean up when destroying the node."""
        if self._check_timer is not None:
            self._check_timer.cancel()
        if self._execution_complete is not None:
            self._execution_complete.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    action_server = TestActionServer()

    # Use MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()

    try:
        # Spin until interrupted
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        action_server.get_logger().info("Server shutting down...")
    finally:
        # Destroy the node explicitly
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
