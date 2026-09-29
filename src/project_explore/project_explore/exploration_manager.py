from enum import Enum, auto
from math import hypot
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .mst_planner import Frontier, MSTPlanner
from .nav2_handler import NavigationResult, Nav2Handler


class ExplorationState(Enum):
    """States used to control the exploration process."""

    INITIALISING = auto()
    REQUESTING_PLAN = auto()
    SELECTING_GOAL = auto()
    NAVIGATING = auto()
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
        self.mst_planner = MSTPlanner(self)
        self.nav2_handler = Nav2Handler()

        # Stores the current asynchronous frontier request while it finishes.
        self.frontier_future = None

        self.ordered_frontiers: List[Frontier] = []
        self.visited_positions: List[Tuple[float, float]] = []
        self.failed_positions: List[Tuple[float, float]] = []
        self.current_goal: Optional[Frontier] = None
        self.last_navigation_result: Optional[NavigationResult] = None

        # Treat nearby goals as the same frontier so they are not retried.
        self.position_tolerance = 0.50
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

    def request_plan(self) -> None:
        """Request the latest frontiers without blocking the node."""

        if self.frontier_future is not None:
            return

        future = self.mst_planner.receive_frontiers()

        if future is None:
            self.get_logger().warning("Frontier service is not ready.")
            return

        self.frontier_future = future
        self.get_logger().info("Requested current frontiers.")

    def process_frontier_response(self) -> bool:
        """Use the frontier response once the asynchronous request is finished."""

        if self.frontier_future is None or not self.frontier_future.done():
            return False

        robot_position = self.get_robot_position()

        if robot_position is None:
            # Keep the finished request and try the TF lookup again next cycle.
            return False

        try:
            response = self.frontier_future.result()
            # Check frontier search response for completion
            if self.mst_planner.completion_check(response):
                # Frontier message marked search as complete
                self.state = ExplorationState.COMPLETE
                return False            
            frontiers = self.mst_planner.parse_frontier_response(response)
            _, ordered_frontiers = self.mst_planner.generate_latest_plan(frontiers, robot_position)
        except (ValueError, RuntimeError) as error:
            # Clear the failed request so a fresh set of frontiers can be requested.
            self.get_logger().warning(f"Frontier plan not ready: {error}. Retrying...")
            self.frontier_future = None
            self.state = ExplorationState.REQUESTING_PLAN
            return False

        # Only store the plan after ordered_frontiers has been successfully created.
        self.ordered_frontiers = ordered_frontiers
        self.frontier_future = None
        self.get_logger().info(f"MST plan generated with {len(self.ordered_frontiers)} frontier(s).")
        return True

    @staticmethod
    def distance(position_a: Tuple[float, float], position_b: Tuple[float, float]) -> float:
        """Calculate the straight-line distance between two positions."""

        return hypot(position_b[0] - position_a[0], position_b[1] - position_a[1])

    def position_recorded(self, frontier: Frontier, recorded_positions: List[Tuple[float, float]]) -> bool:
        """Check whether this frontier is close to a recorded position."""

        frontier_position = (frontier.x, frontier.y)
        return any(
            self.distance(frontier_position, recorded_position) <= self.position_tolerance
            for recorded_position in recorded_positions
        )

    def select_goal(self, ordered_frontiers: List[Frontier]) -> Optional[Frontier]:
        """Select the first frontier that has not been reached or failed."""

        for frontier in ordered_frontiers:
            if self.position_recorded(frontier, self.visited_positions):
                continue

            if self.position_recorded(frontier, self.failed_positions):
                continue

            return frontier

        return None

    def start_navigation(self) -> None:
        """Send the selected frontier to Nav2."""

        if self.current_goal is None:
            self.state = ExplorationState.REQUESTING_PLAN
            return

        self.get_logger().info(
            f"Sending frontier {self.current_goal.frontier_id}: "
            f"({self.current_goal.x:.2f}, {self.current_goal.y:.2f})"
        )

        goal_sent = self.nav2_handler.navigate_to(
            self.current_goal.x,
            self.current_goal.y,
            self.navigation_finished,
        )

        if goal_sent:
            self.navigation_started = True
            return

        self.last_navigation_result = NavigationResult.FAILED
        self.state = ExplorationState.UPDATING_RECORDS

    def navigation_finished(self, result: NavigationResult) -> None:
        """Receive the final result from Nav2."""

        self.last_navigation_result = result
        self.navigation_started = False
        self.state = ExplorationState.UPDATING_RECORDS

    def update_records(self) -> None:
        """Record whether the current frontier was reached or failed."""

        if self.current_goal is None:
            return

        goal_position = (self.current_goal.x, self.current_goal.y)

        if self.last_navigation_result == NavigationResult.SUCCEEDED:
            self.visited_positions.append(goal_position)
            self.get_logger().info(f"Reached frontier {self.current_goal.frontier_id}.")
        else:
            self.failed_positions.append(goal_position)
            self.get_logger().warning(f"Failed to reach frontier {self.current_goal.frontier_id}.")

        self.current_goal = None
        self.last_navigation_result = None

    def publish_mission_status(self) -> None:
        """Report that exploration has finished."""

        self.get_logger().info("Exploration complete.")

    def step(self) -> None:
        """Run the current stage of the exploration process."""

        if self.state == ExplorationState.INITIALISING:
            if self.dependencies_available():
                self.state = ExplorationState.REQUESTING_PLAN

        elif self.state == ExplorationState.REQUESTING_PLAN:
            if self.frontier_future is None:
                self.request_plan()
            elif self.process_frontier_response():
                self.state = ExplorationState.SELECTING_GOAL

        elif self.state == ExplorationState.SELECTING_GOAL:
            self.current_goal = self.select_goal(self.ordered_frontiers)

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

        elif self.state == ExplorationState.UPDATING_RECORDS:
            self.update_records()

            # The map and frontiers may have changed, so generate a fresh MST plan.
            self.state = ExplorationState.REQUESTING_PLAN

        elif self.state == ExplorationState.COMPLETE:
            if not self.mission_complete:
                self.publish_mission_status()
                self.mission_complete = True
                self.timer.cancel()
                rclpy.shutdown()


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
