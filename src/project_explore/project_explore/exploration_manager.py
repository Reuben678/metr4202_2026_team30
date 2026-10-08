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

MAX_RECOVERY_CALLS = 3 # Allowed recovery calls per navigation goal

SHUTDOWN_TIMEOUT = 5.0

RECOVERY_TIMEOUT = 60

class ExplorationState(Enum):
    """States used to control the exploration process."""

    INITIALISE = auto()
    REQUEST_PLAN = auto()
    SELECT_GOAL = auto()
    EXPLORATION = auto()
    TRAVERSAL = auto()
    RECOVERY = auto()
    COMPLETE = auto()
    

class ExplorationManager(Node):
    """Manage frontier planning and navigation."""

    def __init__(self) -> None:
        super().__init__("exploration_manager")

        self.state = ExplorationState.INITIALISE
        
        # Used to find the robot's current position on the generated map.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # The planner uses this node to request frontiers from Frontier Search.
        self.graph_manager = GraphManager(self)
        self.nav2_handler = Nav2Handler()

        # Navigation
        self.current_goal: Optional[Frontier] = None
        self.traversal_path: List[Frontier] = []
        self.navigation_mode = ExplorationState.EXPLORATION
        self.navigation_started = False
        self.last_navigation_result: Optional[NavigationResult] = None

        # recovery
        self.recovery_start_time = None
        self.recovery_future = None
        self.recovery_outcome: Optional[bool] = None
        self.recovery_calls = 0


        # Completion
        self.completion_futures = []
        self.completion_start_time = None
        self.completion_requested = False
        self.mission_complete = False
        self.dependencies_initialised = False

        # Create timer _________________________________________________________
        self.timer = self.create_timer(1.0, self.step)

        # Create client ________________________________________________________
        self.recovery_client = self.create_client(
                RecoveryTrigger,
                '/recovery_node_service'
                )
        
        while not self.recovery_client.wait_for_service(timeout_sec = 10.0):
            self.get_logger().warn("RecoveryTrigger service is not ready, waiting")
        
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

    # Planning state ___________________________________________________________
    def request_plan(self) -> None:
        
        if not self.graph_manager.request_frontiers():
            return False

        try:
            node_ids = self.graph_manager.process_frontier_response()
        except (ValueError, RuntimeError) as error:
            self.get_logger().warning(f"Frontier response not usable, retrying {error}")
            return False
        
        if node_ids is None:
            return False
        
        self.get_logger().info(
            f"{len(node_ids)} frontier node(s) added or updated at node"
            f"{self.graph_manager.current_node_id}"
        )
        return True

    def select_next(self) -> None:

        goal = self.graph_manager.select_goal()

        if goal is not None:
            self.begin_navigation(goal, ExplorationState.EXPLORATION)
            return
        
        path = self.graph_manager.plan_traversal()

        if path:
            self.traversal_path = path
            self.get_logger().info(
                f"No local frontiers, traversing {len(path)} node(s) to node {path[-1].frontier_id}"
            )
            self.begin_navigation(self.traversal_path[0], ExplorationState.TRAVERSAL)
            return

        self.state = ExplorationState.COMPLETE

    # Navigation _______________________________________________________________
    def begin_navigation(self, goal: Frontier, mode: ExplorationState) -> None:

        self.current_goal = goal
        self.navigation_mode = mode
        self.navigation_started = False
        self.last_navigation_result = None
        self.recovery_calls = 0
        self.state = mode
    
    def start_navigation(self) -> None:

        self.get_logger().info(
            f"Sending node {self.current_goal.frontier_id}:"
            f"({self.current_goal.x:.2f}, {self.current_goal.y:.2f})"
        )

        goal_sent = self.nav2_handler.navigate_to(
            self.current_goal.x,
            self.current_goal.y,
            self.navigation_finished,
            self.get_robot_position()
        )

        if goal_sent:
            self.navigation_started = True
            return
        
        self.last_navigation_result = NavigationResult.FAILED

    def navigation_finished(self, result: NavigationResult) -> None:

        self.last_navigation_result = result
    
    def drive_navigation(self) -> None:

        if self.last_navigation_result is not None:
            result = self.last_navigation_result
            self.last_navigation_result = None
            self.navigation_started = False
        
            if result == NavigationResult.SUCCEEDED:
                self.goal_reached()
            else:
                self.navigation_failed()
            return
        
        if not self.navigation_started:
            self.start_navigation()
        else:
            self.nav2_handler.update()
        
    def goal_reached(self) -> None:

        self.get_logger().info(f"Reached node {self.current_goal.frontier_id}")
        self.graph_manager.mark_reached(self.current_goal.frontier_id)

        if self.navigation_mode == ExplorationState.TRAVERSAL:
            self.traversal_path.pop(0)

            if self.traversal_path:
                self.begin_navigation(self.traversal_path[0], ExplorationState.TRAVERSAL)
                return
        
        self.current_goal = None
        self.state = ExplorationState.REQUEST_PLAN
        
    def navigation_failed(self) -> None:

        if self.recovery_calls < MAX_RECOVERY_CALLS:
            self.recovery_calls += 1
            self.state = ExplorationState.RECOVERY
            return
        
        self.abandon_goal()
    
    def abandon_goal(self) -> None:

        if self.navigation_mode == ExplorationState.TRAVERSAL and self.traversal_path:
            target = self.traversal_path[-1]
        else:
            target = self.current_goal
        
        if target is not None:
            self.get_logger().warning(f"Failed to reach node {target.frontier_id}")
            self.graph_manager.remove_node(target.frontier_id)
        
        self.current_goal = None
        self.traversal_path = []
        self.state = ExplorationState.SELECT_GOAL

    # Recovery _________________________________________________________________
    def trigger_recovery(self) -> None:
        if self.recovery_future is not None:
            return # Request already called
        
        if not self.recovery_client.service_is_ready():
            self.get_logger().warn("RecoveryHelper service is not ready")
            self.recovery_outcome = False
            return
        
        request = RecoveryTrigger.Request()
        request.complete = False

        self.recovery_future = self.recovery_client.call_async(request)
        self.recovery_future.add_done_callback(self.recovery_resp_callback)
        self.recovery_start_time = self.get_clock().now()
        self.get_logger().info("Recovery node called, awaiting reply...")

    def recovery_resp_callback(self, future) -> None:
        if future is not self.recovery_future:
            # Late recovery reply, already timedout
            return

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"RecoveryTrigger request failed: {error}")
            self.recovery_outcome = False
            self.recovery_future = None
            return
        
        if response is None:
            self.get_logger().warning("RecoveryTrigger returned no response")
            self.recovery_outcome = False
        else:
            self.get_logger().info(f"Recovery replied: success={response.success}")
            self.recovery_outcome = response.success

        self.recovery_future = None

    # Completion _______________________________________________________________
    def notify_completion(self) -> None:

        futures = []

        frontier_future = self.graph_manager.send_end_search()
        if frontier_future is not None:
            futures.append(frontier_future)
        
        # Recovery node
        if self.recovery_client.service_is_ready():
            request = RecoveryTrigger.Request()
            request.complete = True
            futures.append(self.recovery_client.call_async(request))
        else:
            self.get_logger().warning("RecoveryHelper service not available")
        
        self.completion_futures = futures
        self.completion_start_time = self.get_clock().now()
    
    def publish_mission_status(self) -> None:
        self.get_logger().info("Exploration complete")

    # State Machine ____________________________________________________________

    def step(self) -> None:
        """Run the current stage of the exploration process."""

        match self.state:
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.INITIALISE: # Initialise exploration node
                if not self.dependencies_available():
                    return
                
                robot_position = self.get_robot_position()
                if robot_position is None:
                    return
                
                self.graph_manager.initialise(robot_position)

                self.state = ExplorationState.REQUEST_PLAN
   
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.REQUEST_PLAN: # Request exploration plan
                if self.request_plan():
                    self.state = ExplorationState.SELECT_GOAL

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.SELECT_GOAL: # Select exploration frontier
                self.select_next()

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.EXPLORATION: # Explore with navigation
                self.drive_navigation()

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.TRAVERSAL: # Traverse graph to new frontier
                self.drive_navigation()

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.RECOVERY: # Call recovery node
                if self.recovery_outcome is None:
                    if self.recovery_future is None:
                        self.get_logger().info("Calling recovery")
                        self.trigger_recovery()
                    else:
                        elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
                        if elapsed > RECOVERY_TIMEOUT:
                            self.get_logger().warn("Recovery timed out")
                            self.recovery_future = None
                            self.recovery_outcome = False
                    return
                
                outcome = self.recovery_outcome
                self.recovery_outcome = None

                if outcome:
                    self.get_logger().info("Recovery success, navigating...")
                    self.state = self.navigation_mode
                else:
                    self.get_logger().warn("Recovery failed, goal abandoned")
                    self.abandon_goal()

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            case ExplorationState.COMPLETE: # Mission complete, shutdown
                if self.mission_complete:
                    return
                
                if not self.completion_requested:
                    self.notify_completion()
                    self.completion_requested = True
                    return
                
                elapsed = (self.get_clock().now() - self.completion_start_time).nanoseconds / 1e9
                acknowledged = all(future.done() for future in self.completion_futures)

                if acknowledged or elapsed > SHUTDOWN_TIMEOUT:
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
