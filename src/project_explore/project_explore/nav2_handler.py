from enum import Enum, auto
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter


class NavigationResult(Enum):
    """Results returned to the Exploration Manager."""

    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()


class Nav2Handler:
    """Send navigation goals and report their results using Nav2."""

    def __init__(self) -> None:
        self.navigator = BasicNavigator()

        # BasicNavigator creates its own ROS node, so it must also use Gazebo time.
        self.navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.navigation_start_time = None
        self.navigation_timeout = 45.0
        self.navigation_active = False
        self.result_callback: Optional[Callable[[NavigationResult], None]] = None

    def wait_until_active(self) -> None:
        """Wait until Nav2 is ready to receive a goal."""

        self.navigator.get_logger().info("Waiting for NavigateToPose action server...")

        while not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.navigator.get_logger().info("NavigateToPose action server not available, waiting...")

        self.navigator.get_logger().info("NavigateToPose action server is available.")

    def navigate_to(self, x: float, y: float, result_callback: Callable[[NavigationResult], None]) -> bool:
        """Send a map position to Nav2 as the next navigation goal."""

        if self.navigation_active:
            self.navigator.get_logger().warning("A navigation goal is already active.")
            return False

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # No particular final heading is required, so use a neutral orientation.
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.result_callback = result_callback
        self.navigation_active = True

        # Start a new timeout for this goal, not when the handler is created.
        self.navigation_start_time = self.navigator.get_clock().now()
        self.navigator.goToPose(goal)
        self.navigator.get_logger().info(f"Navigation goal sent: ({x:.2f}, {y:.2f})")
        return True

    def update(self) -> None:
        """Check whether navigation has finished or timed out."""

        if not self.navigation_active:
            return

        # Check the timeout while navigation is active so a stuck goal is cancelled.
        if self.navigation_start_time is not None:
            elapsed = (self.navigator.get_clock().now() - self.navigation_start_time).nanoseconds / 1e9

            if elapsed > self.navigation_timeout:
                self.navigator.get_logger().warning("Navigation timed out.")
                self.navigator.cancelTask()
                self.finish_navigation(NavigationResult.FAILED)
                return

        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            if feedback is not None:
                self.navigator.get_logger().debug(f"Distance remaining: {feedback.distance_remaining:.2f} m")

            return

        task_result = self.navigator.getResult()

        if task_result == TaskResult.SUCCEEDED:
            result = NavigationResult.SUCCEEDED
            self.navigator.get_logger().info("Destination reached.")
        elif task_result == TaskResult.CANCELED:
            result = NavigationResult.CANCELLED
            self.navigator.get_logger().warning("Navigation was cancelled.")
        else:
            result = NavigationResult.FAILED
            self.navigator.get_logger().warning("Navigation failed.")

        self.finish_navigation(result)

    def finish_navigation(self, result: NavigationResult) -> None:
        """Clear the current goal and report its result to the Exploration Manager."""

        self.navigation_active = False
        self.navigation_start_time = None

        callback = self.result_callback
        self.result_callback = None

        if callback is not None:
            callback(result)

    def cancel_goal(self) -> None:
        """Cancel the current navigation goal."""

        if not self.navigation_active:
            return

        self.navigator.get_logger().warning("Cancelling current navigation goal.")
        self.navigator.cancelTask()

    def destroy(self) -> None:
        """Destroy the BasicNavigator ROS node."""

        self.navigator.destroy_node()
