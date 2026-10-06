from enum import Enum, auto
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import DriveOnHeading, Spin
from action_msgs.msg import GoalStatus
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

import math

# Recovery Macros
MIN_RADIUS = 8
MAX_RADIUS = 12
RADIAL_STEP = 1
PHI_STEP = 15
RECOVERY_ATTEMPS = 3
RECOVERY_PERIOD = 5

# Drive Parameters
DRIVE_SPEED = 0.05
DRIVE_TIMEOUT = 15
SPIN_TIMEOUT = 10

class NavigationResult(Enum):
    """Results returned to the Exploration Manager."""

    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()

class Nav2Handler:
    """Send navigation goals and report their results using Nav2."""

    def __init__(self) -> None:
        
        self.navigator = BasicNavigator()
        # BasicNavigator creates its own ROS node -> use Gazebo time.
        self.navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.navigation_start_time = None
        self.navigation_timeout = 45.0
        self.navigation_active = False
        self.result_callback: Optional[Callable[[NavigationResult], None]] = None

    # Class methods __________________________________________________________
    def wait_until_active(self) -> None:
        """Wait until Nav2 is ready to receive a goal."""

        self.navigator.get_logger().info("Waiting for NavigateToPose action server...")

        while not self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
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

        px, py = get_robot_pose()
        # Determine an approx orientation we will end in
        orientation = math.atan2((py - y), (px - x))

        # End in an orientation in that aligns with direction
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = orientation

        self.result_callback = result_callback
        self.navigation_active = True

        # Start a new timeout for this goal, not when the handler is created.
        self.navigation_start_time = self.navigator.get_clock().now()
        self.navigator.goToPose(goal)
        return True

    def update(self) -> None:
        """Check whether navigation has finished or timed out."""

        if not self.navigation_active:
            return

        # Check the timeout while navigation is active so a stuck goal is cancelled.
        if self.navigation_start_time is not None:
            elapsed = (self.navigator.get_clock().now() - 
                        self.navigation_start_time).nanoseconds / 1e9

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

            pass
        self.finish_navigation(result)

    def finish_navigation(self, result: NavigationResult) -> None:
        """Clear the current goal and report its result to the Exploration Manager."""

        self.navigation_active = False
        self.navigation_start_time = None

        callback = self.result_callback
        self.result_callback = None

        self.current_goal = None

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
