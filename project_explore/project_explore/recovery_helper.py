"""
recovery_helper.py
Author: Mitchell Crawford (s4584081)
METR4202, Sem2, 2026
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav2_msgs.action import DriveOnHeading, Spin

from metr4202_interfaces.srv import RecoveryTrigger


# Defined Macros
RECOVERY_ATTEMPTS = 3       # Spin + drive attempts per recovery request
RECOVERY_PERIOD = 10        # Time allowance for a spin [s]
SERVER_TIMEOUT = 5.0        # Wait for a Nav2 behaviour server [s]
MIN_RADIUS = 0.3            # Minimum recovery drive distance [m]
MAX_RADIUS = 1.0            # Maximum recovery drive distance [m]
RADIAL_STEP = 0.1           # Spacing of costmap samples along a bearing [m]
YAW_STEP = 15               # Spacing of bearings searched [deg]
DRIVE_SPEED = 0.05          # Recovery drive speed [m/s]
DRIVE_MARGIN = 5            # Extra time allowance on top of distance / speed [s]
SHUTDOWN_DELAY = 1.0        # Delay after replying before shutdown [s]

# Costmap values (nav_msgs/OccupancyGrid scaling of the Nav2 costmap)
UNKNOWN_COST = -1
INSCRIBED_COST = 99
LETHAL_COST = 100
MAX_TRAVERSABLE_COST = 98   # Highest cost the recovery drive may pass through

class RecoveryHelper(Node):
    def __init__(self):
        super().__init__('RecoveryHelper')

        # Class variables ______________________________________________________
        self.latest_odom = None
        self.latest_costmap = None
        self.recovering = False
        self.shutdown_timer = None

        # The service callback waits on action results, so the service and the
        # action clients share a reentrant group and the node runs on a
        # MultiThreadedExecutor (see main). Otherwise the callback deadlocks.
        self.cb_group = ReentrantCallbackGroup()

        # Create action clients ________________________________________________
        self.spin_client = ActionClient(
                self, Spin, 'spin',
                callback_group=self.cb_group
                )
        self.drive_client = ActionClient(
                self, DriveOnHeading, 'drive_on_heading',
                callback_group=self.cb_group
                )

        # Create subscriptions _________________________________________________
        self.sub_odom = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
                )

        self.sub_costmap = self.create_subscription(
                OccupancyGrid,
                '/local_costmap/costmap',
                self.costmap_callback,
                10
                )

        # Create service _______________________________________________________
        self.recovery_srv = self.create_service(
                RecoveryTrigger,
                '/recovery_node_service',
                self.recovery_callback,
                callback_group=self.cb_group
                )
        
        self.get_logger().info("RecoveryHelper node intialised")

    # Callback functions _______________________________________________________
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def costmap_callback(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def recovery_callback(self, request, response):

        if request.complete:
            # Reply first, then shut down shortly after so the reply is delivered
            self.get_logger().info("Exploration complete, RecoveryHelper shutting down...")
            response.success = True
            if self.shutdown_timer is None:
                self.shutdown_timer = self.create_timer(SHUTDOWN_DELAY, self.shutdown_callback)
            return response

        if self.recovering:
            self.get_logger().warn("Recovery already in progress, rejecting request")
            response.success = False
            return response

        # Trigger a manual failure recovery
        self.recovering = True
        try:
            outcome = self.failure_recovery()
        finally:
            self.recovering = False

        if outcome:
            self.get_logger().info("Successfully recovered robot")
        else:
            self.get_logger().info("Failed to recover robot")
        response.success = outcome
        return response

    def shutdown_callback(self):
        self.shutdown_timer.cancel()
        rclpy.shutdown()

    # Node Methods _____________________________________________________________
    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Robot (x, y, yaw) in the odom frame."""
        if self.latest_odom is None:
            self.get_logger().warn("Could not gather pose from odom")
            return None
        px = self.latest_odom.pose.pose.position.x
        py = self.latest_odom.pose.pose.position.y
        qw = self.latest_odom.pose.pose.orientation.w
        qz = self.latest_odom.pose.pose.orientation.z
        pyaw = 2 * math.atan2(qz, qw)
        return (px, py, pyaw)

    def get_local_costmap(self) -> Optional[OccupancyGrid]:
        if self.latest_costmap is None:
            self.get_logger().warn("Failed to get local costmap data")
            return None
        return self.latest_costmap

    def failure_recovery(self) -> bool:

        self.get_logger().warn("### Failure recovery called ###")

        # Get current data
        robot = self.get_robot_pose()
        costmap = self.get_local_costmap()

        if robot is None:
            self.get_logger().error(f"Attempt failed due to odom")
            return False
        if costmap is None:
            self.get_logger().error(f"Attempt failed due to costmap")
            return False

        # Odom pose and local costmap must share a frame for the search to be valid
        odom_frame = self.latest_odom.header.frame_id
        if costmap.header.frame_id != odom_frame:
            self.get_logger().warn(
                f"Local costmap frame '{costmap.header.frame_id}' differs from "
                f"odom frame '{odom_frame}', recovery pose may be offset"
            )

        rx, ry, ryaw = robot
        info = costmap.info
        grid = np.array(costmap.data, dtype=np.int8).reshape((info.height, info.width))

        target = self.get_recovery_pose((rx, ry), grid, info)
        if target is None:
            self.get_logger().warn("No clear direction found in the local costmap")
            return False

        gx, gy = target
        self.get_logger().info(f"Recovery target ({gx:.2f}, {gy:.2f})")

        if not self.manual_spin(rx, ry, ryaw, (gx, gy)):
            self.get_logger().warn("Spin failed")
            return False

        if not self.manual_drive(rx, ry, (gx, gy)):
            self.get_logger().warn("Drive failed")
            return False

        return True

        self.get_logger().warn(f"Recovery failed after attempt")
        return False
    
    def manual_spin(self, px, py, pyaw, goal_pose) -> bool:
        """Spin in place to face goal_pose."""

        goal_yaw = math.atan2(goal_pose[1] - py, goal_pose[0] - px)
        # Wrap to [-pi, pi] so the robot takes the shorter turn
        delta_yaw = math.atan2(math.sin(goal_yaw - pyaw), math.cos(goal_yaw - pyaw))

        if not self.spin_client.wait_for_server(timeout_sec=SERVER_TIMEOUT):
            self.get_logger().warn("Spin server is not ready")
            return False

        goal = Spin.Goal()
        goal.target_yaw = float(delta_yaw)      # Relative to the current heading
        goal.time_allowance = Duration(sec=RECOVERY_PERIOD)

        self.get_logger().info(f"Spin action sent for {math.degrees(delta_yaw):.1f} deg")

        # Blocks this callback until the result; safe with the MultiThreadedExecutor
        result = self.spin_client.send_goal(goal)

        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED

    def manual_drive(self, px, py, goal_pose) -> bool:
        """Drive forward along the current heading to goal_pose."""

        if not self.drive_client.wait_for_server(timeout_sec=SERVER_TIMEOUT):
            self.get_logger().warn("Drive_on_heading server not ready")
            return False

        distance = math.dist((px, py), goal_pose)

        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance)         # Forward, the robot already faces the target
        goal.speed = DRIVE_SPEED
        # Allow enough time to cover the distance at DRIVE_SPEED
        goal.time_allowance = Duration(sec=int(math.ceil(distance / DRIVE_SPEED)) + DRIVE_MARGIN)

        self.get_logger().info(f"Drive action sent for {distance:.2f} m")

        result = self.drive_client.send_goal(goal)

        return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
    
    def world_to_grid(self, wx, wy, info) -> Optional[Tuple[int, int]]:
        ox, oy = info.origin.position.x, info.origin.position.y

        gx = math.floor((wx - ox) / info.resolution)
        gy = math.floor((wy - oy) / info.resolution)

        if (0 <= gx < info.width) and (0 <= gy < info.height):
            return gx, gy
        return None

    def grid_to_world(self, gx, gy, info) -> Tuple[float, float]:
        
        wx = info.origin.position.x + (gx + 0.5) * info.resolution
        wy = info.origin.position.y + (gy + 0.5) * info.resolution

        return wx, wy

    def ray_cost(self, origin, yaw, radius, grid, info) -> int:
        """Highest cost along the ray from origin out to radius along yaw."""

        worst = 0
        steps = int(round(radius / RADIAL_STEP))
        for step in range(1, steps + 1):
            r = step * RADIAL_STEP
            worst = max(worst, self.get_cost(
                (origin[0] + r * math.cos(yaw), origin[1] + r * math.sin(yaw)), grid, info
            ))
        return worst

    def get_recovery_pose(self, pose, grid, info) -> Optional[Tuple[float, float]]:
        """
        Search bearings for a traversable path of at least MIN_RADIUS, march each one
        out towards MAX_RADIUS while it stays traversable, and pick the bearing that
        reaches furthest (ties go to the lowest cost). Returns a world (odom) position.
        """

        # Search bearings from 0 to 2pi
        bearings = np.linspace(0, 2 * np.pi, (360 // YAW_STEP), endpoint=False)

        best = None     # (reach, -cost, yaw)
        for yaw in bearings:
            yaw = float(yaw)

            # Path out to MIN_RADIUS must be traversable
            cost = self.ray_cost(pose, yaw, MIN_RADIUS, grid, info)
            if cost > MAX_TRAVERSABLE_COST:
                continue

            # March radius along this bearing until the path is no longer traversable
            reach = MIN_RADIUS
            radius = MIN_RADIUS + RADIAL_STEP
            while radius <= MAX_RADIUS + 1e-6:
                point = (pose[0] + radius * math.cos(yaw), pose[1] + radius * math.sin(yaw))
                if self.get_cost(point, grid, info) > MAX_TRAVERSABLE_COST:
                    break
                reach = radius
                radius += RADIAL_STEP

            candidate = (reach, -cost, yaw)
            if best is None or candidate > best:
                best = candidate

        if best is None:
            return None

        reach, _, yaw = best
        return (pose[0] + reach * math.cos(yaw),
                pose[1] + reach * math.sin(yaw))

    def get_cost(self, point, grid, info) -> int:
        """Cost of the cell at a world (odom) point; outside the costmap counts as lethal."""

        cell = self.world_to_grid(point[0], point[1], info)
        if cell is None:
            return LETHAL_COST

        gx, gy = cell
        cost = int(grid[gy, gx])        # grid is [row, col] = [y, x]

        # Cost determination
        if cost in (UNKNOWN_COST, INSCRIBED_COST, LETHAL_COST):
            return LETHAL_COST
        return cost

def main():
    rclpy.init()

    recovery_helper = RecoveryHelper()
    executor = MultiThreadedExecutor()
    executor.add_node(recovery_helper)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        recovery_helper.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        
if __name__ == "__main__":
    main()