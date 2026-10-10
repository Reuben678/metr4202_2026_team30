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
    EXPLORING = auto()
    TRAVERSING = auto()
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

        # Retains selected traversal route
        self.traversal_route: List[Frontier] = []

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

    def select_next_goal(self) -> None:
        """Select local exploration or a graph traversal route."""

        # Clear any previous completed traversal.
        self.traversal_route = []
        self.traversal_target = None

        # First preference: explore an unvisited neighbour.
        frontier = self.graph_manager.select_goal()

        if frontier is not None:
            self.current_goal = frontier
            self.state = ExplorationState.EXPLORING

            self.get_logger().info(
                f"Exploring neighbouring frontier "
                f"{frontier.frontier_id}"
            )
            return

        # No local frontier: use Dijkstra to find another area.
        route = self.graph_manager.plan_traversal()

        if not route:
            self.current_goal = None
            self.state = ExplorationState.COMPLETE
            return

        # The last node is the unexplored destination.
        self.traversal_target = route[-1]

        # All preceding nodes must already be visited.
        intermediate_nodes = route[:-1]

        # Until GraphManager is updated in Step 2 of the
        # wider plan, do not traverse an unvisited node
        # while pretending it is a known transit waypoint.
        if any(not node.visited for node in intermediate_nodes):
            self.get_logger().warning(
                "Dijkstra route contains unvisited intermediate "
                "nodes. Replanning as local exploration."
            )

            self.current_goal = route[0]
            self.traversal_target = None
            self.state = ExplorationState.EXPLORING
            return

        if not intermediate_nodes:
            # Destination is directly reachable in the graph.
            self.current_goal = self.traversal_target
            self.traversal_target = None
            self.state = ExplorationState.EXPLORING
            return

        # Store the route rather than recalculating after
        # every successful traversal waypoint.
        self.traversal_route = list(intermediate_nodes)
        self.current_goal = None

        self.get_logger().info(
            f"Traversal planned through "
            f"{len(self.traversal_route)} visited node(s) "
            f"towards frontier {self.traversal_target.frontier_id}."
        )

        self.state = ExplorationState.TRAVERSING

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

        # Record whether we are exploring or traversing.
        self.navigation_mode = self.state

        goal_sent = self.nav2_handler.navigate_to(
            self.current_goal.x,
            self.current_goal.y,
            self.navigation_finished,
            robot_position=robot_position,
        )

        if goal_sent:
            self.navigation_started = True
            return

        # Rejected goals do not trigger physical recovery.
        self.last_navigation_result = NavigationResult.FAILED
        self.state = ExplorationState.UPDATING_RECORDS

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
        """Update the graph and determine the next state."""

        if self.current_goal is None:
            self.state = ExplorationState.REQUESTING_PLAN
            return

        node_id = self.current_goal.frontier_id
        succeeded = (self.last_navigation_result == NavigationResult.SUCCEEDED)

        previous_mode = self.navigation_mode

        if succeeded:
            self.graph_manager.mark_reached(node_id)
            self.get_logger().info(f"Reached graph node {node_id}.")
        else:
            removed = self.graph_manager.remove_node(node_id)

            if not removed:
                self.get_logger().warning(f"Could not remove failed node {node_id}.")

            self.get_logger().warning(f"Failed to reach graph node {node_id}.")

        self.current_goal = None
        self.last_navigation_result = None
        self.navigation_started = False
        self.navigation_mode = None

        if not succeeded:
            # The current route cannot be trusted after failure.
            self.traversal_route = []
            self.traversal_target = None
            self.state = ExplorationState.REQUESTING_PLAN
            return

        if previous_mode == ExplorationState.TRAVERSING:
            # Continue the stored traversal route.
            if self.traversal_route:
                self.state = ExplorationState.TRAVERSING
                return

            # Arrived at the last visited transit node.
            # Now explore the destination frontier.
            if self.traversal_target is not None:
                self.current_goal = self.traversal_target
                self.traversal_target = None
                self.state = ExplorationState.EXPLORING
                return

        # Exploration waypoint reached: obtain fresh frontiers.
        self.state = ExplorationState.REQUESTING_PLAN

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
            self.select_next_goal()

        elif self.state == ExplorationState.EXPLORING:

            if not self.navigation_started:
                self.start_navigation()
            else:
                self.nav2_handler.update()

        elif self.state == ExplorationState.TRAVERSING:

            # Choose the next waypoint from the stored route.
            if self.current_goal is None:
                if self.traversal_route:
                    self.current_goal = self.traversal_route.pop(0)

                    self.get_logger().info(
                        f"Traversing to visited node "
                        f"{self.current_goal.frontier_id}."
                    )
                else:
                    # Defensive fallback.
                    self.state = ExplorationState.REQUESTING_PLAN
                    return

            if not self.navigation_started:
                self.start_navigation()
            else:
                self.nav2_handler.update()

        elif self.state == ExplorationState.RECOVERING:
            self.process_recovery()
    
        elif self.state == ExplorationState.UPDATING_RECORDS:
            self.update_records()

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
