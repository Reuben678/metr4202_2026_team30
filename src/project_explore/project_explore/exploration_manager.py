from enum import Enum, auto
from math import hypot
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .graph_manager import Frontier, GraphManager
from .nav2_handler import NavigationResult, Nav2Handler

from metr4202_interfaces.srv import RecoveryTrigger


class ExplorationState(Enum):
    """States used to control the exploration process."""

    INITIALISING = auto()
    REQUESTING_PLAN = auto()
    SELECTING_GOAL = auto()
    NAVIGATING = auto()
    RECOVERING = auto()
    UPDATING_RECORDS = auto()
    COMPLETE = auto()


class ExplorationManager(Node):
    """Manage frontier planning and navigation."""

    def __init__(self) -> None:
        super().__init__("exploration_manager")

        self.state = ExplorationState.INITIALISING

        # Used to find the robot's current position on the generated map.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # The planner uses this node to request frontiers from Frontier Search.
        self.graph_manager = GraphManager(self)
        self.nav2_handler = Nav2Handler()

        # Client for requesting recovery manoeuvres.
        self.recovery_client = self.create_client(RecoveryTrigger, "/recovery_node_service")

        # Stores the active recovery request.
        self.recovery_future = None

        self.current_goal: Optional[Frontier] = None
        self.last_navigation_result: Optional[NavigationResult] = None

        self.navigation_started = False
        self.mission_complete = False
        self.dependencies_initialised = False

        self.timer = self.create_timer(0.10, self.step)
        self.get_logger().info("Exploration Manager initialised.")

    def dependencies_available(self) -> bool:
        """Wait for Nav2 once before exploration starts."""

        if self.dependencies_initialised:
            return True

        self.get_logger().info("Waiting for Nav2 to become active...")
        self.nav2_handler.wait_until_active()
        self.dependencies_initialised = True
        self.get_logger().info("Nav2 is active.")
        return True

    def get_robot_position(self) -> Optional[Tuple[float, float]]:
        """Get the robot's current map position from TF."""

        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as error:
            self.get_logger().warning(f"Waiting for map -> base_link TF: {error}")
            return None

        translation = transform.transform.translation
        return translation.x, translation.y

    def select_next_goal(self) -> Optional[Frontier]:
        """Choose the next graph node to navigate towards."""

        # First, prioritise unvisited neighbouring frontiers.
        frontier = self.graph_manager.select_goal()

        if frontier is not None:
            self.get_logger().info(f"Selected neighbouring frontier {frontier.frontier_id}")
            return frontier

        # Otherwise, find the shortest route to an unvisited frontier.
        route = self.graph_manager.plan_traversal()

        if route:
            next_node = route[0]

            self.get_logger().info(f"Dijkstra selected graph node {next_node.frontier_id}")

            return next_node

        return None

    def start_navigation(self) -> None:
        """Send the selected frontier to Nav2."""

        if self.current_goal is None:
            self.state = ExplorationState.REQUESTING_PLAN
            return

        # Get the robot's current position to calculate goal orientation.
        robot_position = self.get_robot_position()

        if robot_position is None:
            self.get_logger().warning("Cannot navigate: robot position unavailable.")
            return

        self.get_logger().info(
            f"Sending frontier {self.current_goal.frontier_id}: "
            f"({self.current_goal.x:.2f}, {self.current_goal.y:.2f})"
        )

    def navigation_finished(self, result: NavigationResult) -> None:
        """Receive the navigation result and trigger recovery if needed."""

        self.last_navigation_result = result
        self.navigation_started = False

        if result == NavigationResult.SUCCEEDED:
            self.state = ExplorationState.UPDATING_RECORDS

        else:
            self.get_logger().warning("Navigation failed. Entering recovery state.")
            self.state = ExplorationState.RECOVERING

    def request_recovery(self) -> None:
        """Request a recovery manoeuvre from the Recovery Helper."""

        if self.recovery_future is not None:
            return

        if not self.recovery_client.service_is_ready():
            self.get_logger().warning(
                "Recovery service is not available."
            )
            self.state = ExplorationState.UPDATING_RECORDS
            return

        request = RecoveryTrigger.Request()
        request.complete = False

        self.recovery_future = self.recovery_client.call_async(request)

        self.get_logger().info("Recovery requested.")

    def process_recovery(self) -> None:
        """Check whether the recovery manoeuvre has finished."""

        if self.recovery_future is None:
            self.request_recovery()
            return

        if not self.recovery_future.done():
            return

        try:
            response = self.recovery_future.result()

            if response.success:
                self.get_logger().info("Recovery manoeuvre succeeded.")
            else:
                self.get_logger().warning("Recovery manoeuvre failed.")

        except Exception as error:
            self.get_logger().error(f"Recovery service failed: {error}")

        finally:
            self.recovery_future = None
            self.state = ExplorationState.UPDATING_RECORDS
    
    def update_records(self) -> None:
        """Update the graph after navigation finishes."""

        if self.current_goal is None:
            return

        node_id = self.current_goal.frontier_id

        if self.last_navigation_result == NavigationResult.SUCCEEDED:

            self.graph_manager.mark_reached(node_id)

            self.get_logger().info(
                f"Reached graph node {node_id}."
            )

        else:
            self.graph_manager.remove_node(node_id)

            self.get_logger().warning(
                f"Failed to reach graph node {node_id}."
            )

        self.current_goal = None
        self.last_navigation_result = None
        self.navigation_started = False

    def publish_mission_status(self) -> None:
        """Report that exploration has finished."""

        self.get_logger().info("Exploration complete.")

    def step(self) -> None:
        """Run the current stage of the exploration process."""

        if self.state == ExplorationState.INITIALISING:
            if not self.dependencies_available():
                return

            robot_position = self.get_robot_position()

            if robot_position is None:
                return

            self.graph_manager.initialise(robot_position)

            self.get_logger().info(
                f"Graph initialised at ({robot_position[0]:.2f}, "
                f"{robot_position[1]:.2f})"
                )

            self.state = ExplorationState.REQUESTING_PLAN

        elif self.state == ExplorationState.REQUESTING_PLAN:

            if self.graph_manager.frontier_future is None:
                self.graph_manager.request_frontiers()

            else:
                try:
                    result = self.graph_manager.process_frontier_response()

                except (RuntimeError, ValueError) as error:
                    self.get_logger().warning(f"Frontier update failed: {error}")
                    return

                if result is not None:
                    self.get_logger().info(f"Processed {len(result)} frontier(s).")

                    self.state = ExplorationState.SELECTING_GOAL

        elif self.state == ExplorationState.SELECTING_GOAL:

            self.current_goal = self.select_next_goal()

            if self.current_goal is None:
                self.state = ExplorationState.COMPLETE

            else:
                self.navigation_started = False
                self.state = ExplorationState.NAVIGATING

        elif self.state == ExplorationState.NAVIGATING:
            if not self.navigation_started:
                self.start_navigation()
            else:
                self.nav2_handler.update()

        elif self.state == ExplorationState.RECOVERING:
            self.process_recovery()
    
        elif self.state == ExplorationState.UPDATING_RECORDS:
            self.update_records()

            # The map and frontiers may have changed, so generate a fresh MST plan.
            self.state = ExplorationState.REQUESTING_PLAN

        elif self.state == ExplorationState.COMPLETE:
            if not self.mission_complete:
                self.publish_mission_status()
                self.mission_complete = True
                self.timer.cancel()


def main(args=None) -> None:
    """Start the Exploration Manager node."""

    rclpy.init(args=args)
    manager = ExplorationManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.nav2_handler.destroy()
        manager.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
