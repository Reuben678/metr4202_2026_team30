from enum import Enum, auto
from math import hypot
from typing import List, Optional, Tuple

import time
from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .graph_manager import Frontier, GraphManager
from .nav2_handler import NavigationResult, Nav2Handler

from metr4202_interfaces.srv import RecoveryTrigger

COMPLETION_THRESHOLD = 3
MAX_MAP_RECOVERY_ATTEMPTS = 2
MAP_REFRESH_WAIT = 5.0  # Seconds

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
    VERIFYING_COMPLETION = auto()


class ExplorationManager(Node):
    """Manage frontier planning and navigation."""


    def __init__(self) -> None:
        """Initialise the Exploration Manager and its ROS2 interfaces."""

        super().__init__("exploration_manager")

        # ---------------------------------------------------------
        # Exploration state and mission status
        # ---------------------------------------------------------
        self.state = ExplorationState.INITIALISING
        self.dependencies_initialised = False
        self.navigation_started = False
        self.mission_complete = False

        # ---------------------------------------------------------
        # Graph management and navigation
        # ---------------------------------------------------------
        # Initialise the graph manager and Nav2 navigation handler.
        self.graph_manager = GraphManager(self)
        self.nav2_handler = Nav2Handler()

        # Store the selected graph traversal route and active goal.
        self.traversal_route: List[Frontier] = []
        self.current_goal: Optional[Frontier] = None
        self.last_navigation_result: Optional[NavigationResult] = None

        # Track the last successfully reached graph node.
        self.last_reached_node_id = None

        # ---------------------------------------------------------
        # Robot localisation
        # ---------------------------------------------------------
        # Use TF2 to determine the robot's position in the map frame.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------
        # Frontier and completion monitoring
        # ---------------------------------------------------------
        # Count consecutive frontier updates with no valid goals.
        self.empty_frontier_count = 0

        # Track completion verification following a map refresh.
        self.confirm_after_refresh = False
        self.map_verification_paused = False
        self.map_pause_change_count = 0

        # ---------------------------------------------------------
        # Occupancy map monitoring
        # ---------------------------------------------------------
        # Store the latest map and track received and changed maps.
        self.latest_map = None
        self.last_map_received_time = None
        self.map_message_count = 0
        self.last_map_change_time = None
        self.map_change_count = 0

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10,
        )

        # ---------------------------------------------------------
        # Map recovery and verification
        # ---------------------------------------------------------
        # Track recovery attempts and the current recovery phase.
        self.map_recovery_attempts = 0
        self.map_recovery_phase = "request"

        # Record map status before recovery to detect subsequent updates.
        self.map_change_baseline = 0
        self.map_message_baseline = 0
        self.map_wait_deadline = None

        # ---------------------------------------------------------
        # Recovery service
        # ---------------------------------------------------------
        # Client for requesting robot recovery manoeuvres.
        self.recovery_client = self.create_client(
            RecoveryTrigger,
            "/recovery_node_service",
        )

        # Track the active recovery request and requested behaviour.
        self.recovery_future = None
        self.recovery_type = None

        # ---------------------------------------------------------
        # Exploration execution
        # ---------------------------------------------------------
        # Execute the exploration state machine at 10 Hz.
        self.timer = self.create_timer(0.10, self.step)

        self.get_logger().info("Exploration Manager initialised.")


    def map_callback(self, msg: OccupancyGrid) -> None:
        """Monitor occupancy map messages and detect changes."""

        now = time.monotonic()

        # Record that a new map message has arrived
        self.last_map_received_time = now
        self.map_message_count += 1

        # Determine whether the occupancy grid has changed
        map_changed = (
            self.latest_map is None
            or msg.data != self.latest_map.data
            or msg.info.width != self.latest_map.info.width
            or msg.info.height != self.latest_map.info.height
            or msg.info.resolution != self.latest_map.info.resolution
            or msg.info.origin != self.latest_map.info.origin
        )

        if map_changed:
            self.last_map_change_time = now
            self.map_change_count += 1

        # Store the newest map for the next comparison
        self.latest_map = msg

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
            self.get_logger().warning(
                f"Waiting for map -> base_link TF: {error}"
            )
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
            # Reset completion verification after finding a goal
            self.empty_frontier_count = 0
            self.map_recovery_attempts = 0
            self.confirm_after_refresh = False
            self.map_verification_paused = False

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

            # no valid frontier remains in the graph
            self.empty_frontier_count += 1
            self.get_logger().info(
                f"No valid exploration goals found. "
                f"Completion check "
                f"{self.empty_frontier_count}/{COMPLETION_THRESHOLD}"
            )

            self.current_goal = None

            if self.empty_frontier_count >= COMPLETION_THRESHOLD:

                if self.confirm_after_refresh:

                        self.get_logger().info(
                            "Map refresh completed and no reachable "
                            "frontiers remain. Exploration complete."
                        )

                        self.state = ExplorationState.COMPLETE

                else:

                    self.get_logger().info(
                        "No valid frontiers remain. "
                        "Verifying map before completion."
                    )

                    self.state = ExplorationState.VERIFYING_COMPLETION

            else:
                # Request fresh frontiers before confirming completion.
                self.state = ExplorationState.REQUESTING_PLAN

            return

        # Valid traversal route found, reset completion verification
        self.empty_frontier_count = 0
        self.map_recovery_attempts = 0
        self.confirm_after_refresh = False
        self.map_verification_paused = False

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
            self.get_logger().warning(
                "Cannot navigate: robot position unavailable."
            )
            return

        self.get_logger().info(
            f"Sending frontier {self.current_goal.frontier_id}: "
            f"({self.current_goal.x:.2f}, {self.current_goal.y:.2f})"
        )

        # Record whether we are exploring or traversing.
        self.navigation_mode = self.state

        # For debugging
        self.get_logger().info(
            f"Navigation mode: {self.state.name} | "
            f"Target node: {self.current_goal.frontier_id} | "
            f"Previously visited: {self.current_goal.visited}"
        )

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
            self.get_logger().warning(
                "Navigation failed. Entering recovery state."
            )
            self.recovery_type = "navigation"
            self.state = ExplorationState.RECOVERING

    def request_recovery(self) -> None:
        """Request the selected recovery manoeuvre."""

        if self.recovery_future is not None:
            return

        if not self.recovery_client.service_is_ready():
            self.get_logger().warning("Recovery service is not available.")

            # keep existing navigation recovery behaviour.
            if self.recovery_type == "navigation":
                self.state = ExplorationState.UPDATING_RECORDS

            return

        request = RecoveryTrigger.Request()
        request.complete = False
        request.map_refresh = (self.recovery_type == "map")

        if self.recovery_type == "map":

            # Record map state before performing recovery.
            self.map_change_baseline = self.map_change_count

            self.map_recovery_attempts += 1

            self.get_logger().info(
                f"Map recovery attempt "
                f"{self.map_recovery_attempts}/"
                f"{MAX_MAP_RECOVERY_ATTEMPTS}"
            )

        self.recovery_future = self.recovery_client.call_async(request)

        self.get_logger().info(f"{self.recovery_type} recovery requested.")


    def process_recovery(self) -> None:
        """Process navigation or occupancy-map recovery."""

        # 1. Waiting for a map following recovery
        if (
            self.recovery_type == "map"
            and self.map_recovery_phase == "wait_map"
        ):
            self.wait_for_map_refresh()
            return

        # 2. Send recovery request
        if self.recovery_future is None:
            self.request_recovery()
            return


        # 3. Wait for Recovery Helper response
        if not self.recovery_future.done():
            return

        try:
            response = self.recovery_future.result()
            succeeded = response is not None and response.success

        except Exception as error:
            self.get_logger().error(f"Recovery service failed: {error}")
            succeeded = False

        self.recovery_future = None

        # 4. Handle map recovery
        if self.recovery_type == "map":

            if succeeded:

                self.get_logger().info(
                    "Map recovery rotation completed. "
                    "Waiting for updated occupancy map."
                )

                # Record the messages received at spin completion.
                self.map_message_baseline = self.map_message_count

                # Allow time for SLAM to publish another map.
                self.map_wait_deadline = (time.monotonic() + MAP_REFRESH_WAIT)

                self.map_recovery_phase = "wait_map"

            else:

                self.get_logger().warning("Map recovery rotation failed.")

                self.map_recovery_phase = "request"
                self.state = ExplorationState.VERIFYING_COMPLETION

            return

        # 5. Handle existing navigation recovery
        if succeeded:
            self.get_logger().info("Navigation recovery manoeuvre succeeded.")
        else:
            self.get_logger().warning("Navigation recovery manoeuvre failed.")

        self.recovery_type = None
        self.state = ExplorationState.UPDATING_RECORDS

    def wait_for_map_refresh(self) -> None:
        """Wait for SLAM, then request fresh frontiers after map recovery."""

        # Give SLAM time to process the rotation.
        if time.monotonic() < self.map_wait_deadline:
            return

        # Check whether a fresh map was published.
        new_message = (self.map_message_count > self.map_message_baseline)

        # Check whether occupancy information actually changed.
        map_changed = (self.map_change_count > self.map_change_baseline)

        if new_message and map_changed:
            self.get_logger().info(
                "Occupancy map changed following recovery. "
                "Requesting fresh frontiers."
            )

        elif new_message:
            self.get_logger().info(
                "Occupancy map updated but data unchanged. "
                "Checking for remaining frontiers."
            )

        else:
            self.get_logger().warning(
                "No new occupancy map received after recovery. "
                "Rechecking available frontiers."
            )

        # Rotation succeeded and a fresh map message was received.
        # The occupancy values do not need to change.
        self.confirm_after_refresh = new_message

        # Request a new set of frontiers regardless of map changes.
        self.empty_frontier_count = 0
        self.map_recovery_phase = "request"
        self.recovery_type = None

        self.state = ExplorationState.REQUESTING_PLAN

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
            self.last_reached_node_id = node_id

            self.get_logger().info(f"Reached graph node {node_id}.")

        else:
            if previous_mode == ExplorationState.TRAVERSING:
                # Keep previously visited nodes and their graph connections.
                self.get_logger().warning(
                    f"Failed traversal to visited node {node_id}. "
                    "Preserving graph connectivity."
                )

            else:
                # Record the failed frontier's map coordinates so it
                # cannot be rediscovered and immediately selected again.
                self.graph_manager.blacklist_frontier(
                    self.current_goal.position
                )

                # Remove the failed frontier node from the graph.
                removed = self.graph_manager.remove_node(node_id)

                if not removed:
                    self.get_logger().warning(
                        f"Could not remove failed node {node_id}."
                    )

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
        markers = self.graph_manager.aruco_nodes

        if not markers:
            self.get_logger().info(
                "ArUco findings: No confirmed markers detected."
            )
            return

        lines = [
            f"ARUCO FINDINGS SUMMARY ({len(markers)} markers)",
            "Coordinates in map frame:"
        ]

        for marker_id, marker in sorted(markers.items()):
            lines.append(
                f"  ArUco ID {marker_id}: "
                f"X = {marker['x']:.2f} m, "
                f"Y = {marker['y']:.2f} m"
            )

        self.get_logger().info("\n".join(lines))

    def step(self) -> None:
        """Run the current stage of the exploration process."""

        if self.state == ExplorationState.INITIALISING:
            if not self.dependencies_available():
                return

            robot_position = self.get_robot_position()

            if robot_position is None:
                return

            self.graph_manager.initialise(robot_position)
            self.last_reached_node_id = 0

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

        elif self.state == ExplorationState.VERIFYING_COMPLETION:

            if self.map_verification_paused:

                # If the map eventually changes, resume planning.
                if self.map_change_count > self.map_pause_change_count:

                    self.get_logger().info(
                        "New map information detected. Resuming exploration."
                    )

                    self.map_verification_paused = False
                    self.map_recovery_attempts = 0
                    self.empty_frontier_count = 0
                    self.confirm_after_refresh = False

                    self.state = ExplorationState.REQUESTING_PLAN

            elif self.map_recovery_attempts >= MAX_MAP_RECOVERY_ATTEMPTS:

                # Check whether any unexplored graph nodes remain.
                remaining_frontiers = [
                    node_id
                    for node_id in self.graph_manager.graph.nodes
                    if not self.graph_manager.get_node(node_id).visited
                ]

                # Check whether SLAM has published a recent map.
                map_is_recent = (
                    self.last_map_received_time is not None
                    and time.monotonic() - self.last_map_received_time <= 10.0
                )

                if (
                    not remaining_frontiers
                    and map_is_recent
                    and self.empty_frontier_count >= COMPLETION_THRESHOLD
                ):
                    # Completion should not depend on a successful spin
                    # when no eligible unexplored frontiers remain.
                    self.get_logger().info(
                        "Map verification spin could not be completed, "
                        "but the latest map is recent and no eligible "
                        "unvisited frontiers remain. Exploration complete."
                    )

                    self.state = ExplorationState.COMPLETE

                else:
                    # Unvisited frontiers or stale map: don't claim success.
                    self.get_logger().error(
                        "Map verification failed and exploration "
                        "completion cannot be confirmed. "
                        f"Unvisited graph frontiers: "
                        f"{len(remaining_frontiers)}; "
                        f"recent map available: {map_is_recent}."
                    )

                    self.map_verification_paused = True
                    self.map_pause_change_count = self.map_change_count


            else:

                self.recovery_type = "map"
                self.map_recovery_phase = "request"
                self.state = ExplorationState.RECOVERING

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
