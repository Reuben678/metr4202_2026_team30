from enum import Enum, auto
from math import hypot
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .mst_planner import Frontier, MSTPlanner
from .nav2_handler import NavigationResult, Nav2Handler

from metr4202_interfaces import RecoveryTrigger

class ExplorationState(Enum):
    """States used to control the exploration process."""

    INITIALISE = auto()
    REQUEST_PLAN = auto()
    SELECT_GOAL = auto()
    EXPLORATION = auto()
    TRAVERSAL = auto()
    RECOVERY = auto()
    RECORDS = auto()
    COMPLETE = auto()
    

class ExplorationManager(Node):
    """Manage frontier planning and navigation."""

    def __init__(self) -> None:
        super().__init__("exploration_manager")

        self.state = ExplorationState.INITIALISING
        self.recovery_future = None
        self.recovery_outcome = None

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

        
        # Create timer _________________________________________________________
        self.timer = self.create_timer(0.10, self.step)

        # Create client ________________________________________________________
        self.recovery_client = self.create_client(
                RecoveryTrigger,
                'recovery_node_service'
                )
        
        while not self.recovery_client.wait_for_service(timeout_sec = 10.0):
            self.get_logger().warn("RecoveryTrigger service is not available")
        self.get_logger().info("RecoveryHelper service online")    

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
            self.state = ExplorationState.REQUEST_PLAN
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
            self.state = ExplorationState.REQUEST_PLAN
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
        self.state = ExplorationState.RECORDING

    def navigation_finished(self, result: NavigationResult) -> None:
        """Receive the final result from Nav2."""

        self.last_navigation_result = result
        self.navigation_started = False
        if (result == NavigationResult.FAILED):
            self.state = ExplorationState.RECOVERY
            return
        self.state = ExplorationState.RECORDING

    def update_records(self) -> None:
        """Record whether the current frontier was reached or failed."""

        if self.current_goal is None:
            return

        pose = get_robot_position()

        goal_position = (self.current_goal.x, self.current_goal.y, pose)

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

    def trigger_recovery(self) -> None:

        if not self.recovery_client.service_is_ready():
            self.get_logger().warn("RecoveryHelper service is not ready")
            return

        if (hasattr(self, "recovery_future")
                and self.recovery_future is not None 
                and not self.recovery_future.done())
            request = TriggerRecovery.Request()
            
            self.recovery_future = self.recovery_client.call_async(request)
            self.recovery_future.add_done.callback(self.recovery_resp_callback)

    def recovery_resp_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error("RecoveryTrigger request failed: {error}")
            self.recovery_future = None
            return

        if response is None:
            self.get_logger().warning("RecoveryTrigger returned no response")
            self.recovery_future = None
            return

        self.recovery_outcome = response.success
        self.recovery_future = None
        return

    def step(self) -> None:
        """Run the current stage of the exploration process."""

        match self.state:
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.INITIALISE: # Initialise exploration node
                if self.dependencies_available():
                    self.state = ExplorationState.REQUEST_PLAN
   
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.REQUEST_PLAN: # Request exploration plan
                if self.frontier_future is None:
                    self.request_plan()
                elif self.process_frontier_response():
                    self.state = ExplorationState.SELECT_GOAL

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.SELECT_GOAL: # Select exploration frontier
                self.current_goal = self.select_goal(self.ordered_frontiers)

                if self.current_goal is None:
                    self.state = ExplorationState.COMPLETE
                else:
                    self.navigation_started = False
                    self.state = ExplorationState.NAVIGATING

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.EXPLORATION: # Explore with navigation
                if not self.navigation_started:
                    self.start_navigation()
                else:
                    self.nav2_handler.update()

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.TRAVERSAL: # Traverse graph to new frontier
                # 1) Select closest frontier node which has not been visited
                # 2) Determine path of nodes to reach selected frontier
                # 3) Call navigation traversal along path

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.RECOVERY: # Call recovery node
                self.trigger_recovery()
                if self.recovery_outcome is not None:
                    # Outcome has been updated to bool
                    if self.recovery_outcome:
                        # Recovery succeeded -> Attemp nav again
                        self.state = ExplorationState.EXPLORATION      
                    else:
                        # Recovery failed
                        self.state = ExplorationState.RECORDING

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.RECORDING: # Update and record outcomes
                self.update_records()
                
                self.state = ExplorationState.REQUEST_PLAN

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.COMPLETE: # Mission complete, shutdown
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
