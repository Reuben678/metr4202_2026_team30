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
        self.latest_odom = None
        self.latest_costmap = None
        self.current_goal = None
        
        # Create action client
        self.spin_client = ActionClient(self.recovery_helper, Spin, 'spin')
        self.spin_done = False
        self.spin_success = False

        self.drive_client = ActionClient(self.recovery_helper, DriveOnHeading, 'drive_on_heading')
        self.drive_done = False
        self.drive_success = False

        # Create subscriptions 
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.sub_costmap = self.create_subscription(
            local_costmap,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )

        # BasicNavigator creates its own ROS node, so it must also use Gazebo time.
        self.navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.navigation_start_time = None
        self.navigation_timeout = 45.0
        self.navigation_active = False
        self.result_callback: Optional[Callable[[NavigationResult], None]] = None

    # Odometry callback functions _____________________________________________
    def odom_callback(self, msg: Odometry):
        if msg is None:
            self.get_logger().warn("NavHandler received no new Odom data") 
            return
        self.latest_odom = msg


    def costmap_callback(self, msg: Costmap):
        if msg is None:
            self.get_logger().warn("NavHandler received no new costmap data")
            return
        self.latest_costmap = msg

    # Class methods __________________________________________________________

    """
    Get the robot bearing from the latest odometry data
    """
    def get_robot_orientation(self):
        if self.latest_odom is None:
            self.get_logger().warn("No odometry data recieved")
            return None
        w = self.latest_odom.pose.pose.orientation.w
        return w

    """
    Get the current odometry data, provided by the callback function
    """
    def get_robot_pose(self):
        if self.latest_odom is None:
            self.get_logger().warn("No odometry data received")
            return None

        p = self.latest_odom.pose.pose.position
        return p.x, p.y

    """
    Get the current costmap data, provided by the callback function
    """
    def get_local_costmap(self):
        if self.latest_costmap is None:
            self.get_logger().warn("No new costmap data was received")
            return None
        return costmap

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
        self.navigator.get_logger().info(f"Navigation goal sent: ({x:.2f}, {y:.2f})")
        self.current_goal = (x, y)
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
                # Call Navigation recovery
                success = failure_recovery()
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

    """
    Failure recovery backup for navigation
    """
    def failure_recovery(self) -> bool:
        self.get_logger().warn("Failure recovery called!")
        success = False

        for attempt in range(RECOVERY_ATTEMPTS):
            # Get current data
            px, py = get_robot_pose()
            pw = get_robot_orientation()
            costmap = get_local_costmap()
            
            if (pose or costmap) is None:
                self.get_logger().error("Recovery failed due to missing data")
                return
            
            # Process data 
            info = costmap.info
            grid = np.array(costmap.data, dtype=np.int8)
            grid = grid.reshape((info.height, info.width))

            gx, gy = recovery_pose(px, py, grid, info) # Lowest local cost (X,Y)

            goal = grid_to_world(gx, gy, info)
            
            spin_success = manual_spin(px, py, pw, goal)
            if not spin_success:
                continue
            
            drive_sucess = manual_drive(px, py, pw, goal)
            if not drive_success:
                continue
            
            if drive_success and spin_success:
                return True
        self.get_logger().warn(f"Recovery failed after {RECOVERY_ATTEMPTS}")
        return False

    """
    Manually command action to spin towards desired bearing
    """
    def manual_spin(self, px, py, pw, goal) -> bool:
        
        dx = abs(px) - abs(goal[0])
        dy = abs(py) - abs(goal[1])
        
        phi_goal = atan2(dx, dy)
        delta_yaw = math.atan2(np.sin(phi_goal - pw), np.cos(phi_goal - pw))

        if not self.spin_client.server_is_ready():
            self.get_logger().warn("spin server not ready")
            return False
        goal = Spin.Goal()
        goal.target_yaw = float(delta_yaw)
        goal.time_allowance = Duration(sec=10)

        self.spin_done = False
        self.spin_success = False
        response = self.spin_client.send_goal(goal)
        return (response is not None and 
                response.status == GoalStatus.STATUS_SUCCEEDED)
    
    """
    Manually command action to drive required distance to goal
    """
    def manual_drive(px, py, goal) -> bool:
    
        self.drive_ready =self.drive_client.wait_for_server(timout_sec=10.0)
        
        if not self.drive_ready:
            self.get_logger().warn("Drive_on_heading server not ready")
            return False

        dx = abs(px) - abs(goal[0])
        dy = abs(py) - abs(goal[1])
        dist = np.hypot(dx, dy)
        # Prepare goal
        goal = DriveOnHeading.Goal()
        goal.target.x = float(dist)
        goal.speed = DRIVE_SPEED
        goal.time_allowance = Duration(sec=DRIVE_TIMOUT)

        response = self.drive_client.send_goal(goal)

        return (response is not None and
                response.status == GoalStatus.STATUS_SUCCEEDED)

    def drive_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Drive was rejected")
            self.drive_done = True
            return
        self.drive_success = handle.get_result_async()
        return

    """
    Convert global coordinates to map grid frame
    """
    def world_to_grid(wx, wy, info) -> tuple[int, int]:
        gx = math.floor((wx - info.origin.x.position.x) / info.resolution)
        gy = math.floor((wy - info.origin.y.position.y) / info.resolution)

        if (0 <= gx <= info.width) and (0 <= gy <= info.height):
            return gx, gy
        return None

    """
    Convert grid coordinates to world frame
    """
    def grid_to_world(gx, gy, info) -> tuple[int, int]:
        wx = info.origin.position.x + (gx + 0.5) * info.resolution
        wy = info.origin.position.y + (gy + 0.5) * info.resolution
        return wx, wy

    """
    Determine the optimal recovery path from radial search
    """
    def recovery_pose(px, py, grid, info):
        gx, gy = world_to_grid(px, py, info)

        # Search bearings from 0 to 2Pi
        bearings = np.linspace(0, 2 * np.pi, (360 / PHI_STEP) , endpoint=false)
        ideal_phi = 0
        # Search across bearings
        for phi in bearings:
            count += 1
            xs = gx + MIN_RADIUS * np.cos(phi)
            ys = gy + MIN_RADIUS * np.sin(phi)
        
            cost = get_cost(xs, ys, grid)
            cost = 100 - cost

            ideal_phi = (ideal_phi + (phi * cost)) / 2

        # March radius on ideal bearing
        ideal_radius = MIN_RADIUS
        for radius in range(MIN_RADIUS, MAX_RADIUS, RADIAL_STEP):
            xs = gx + radius * np.cos(ideal_phi)
            ys = gy + radius * np.sin(ideal_phi)

            cost = get_cost(xs, ys, grid)
            cost = 100 - cost;
            ideal_radius = (ideal_radius + (radius * cost)) / 2

        xs = gx + ideal_radius * np.cos(ideal_phi)
        ys = gy + ideal_radius * np.sin(ideal_phi)

        return (xs, ys)
    
    """
    Return corrected cost of point (x,y) within the costmap coordinate frame
    """
    def get_cost(self, px, py, grid) -> int:
        cost = int(grid[py, px])
        # Correct cost for UNKNOWN OR INSCRIBED
        if (cost < 0 or cost >98):
            cost = 100
        return cost

    def cancel_goal(self) -> None:
        """Cancel the current navigation goal."""

        if not self.navigation_active:
            return

        self.navigator.get_logger().warning("Cancelling current navigation goal.")
        self.navigator.cancelTask()

    def destroy(self) -> None:
        """Destroy the BasicNavigator ROS node."""

        self.navigator.destroy_node()
