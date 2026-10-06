"""
recovery_helper.py
Author: Mitchell Crawford (s4584081)
METR4202, Sem2, 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from metr4202_interfaces.srv import RecoveryTrigger

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

from nav2_msgs.action import DriveOnHeading, Spin
from nav2_msgs.msg import BehaviorTreeLog

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

#from metr4202_interfaces.srv import RecoveryTrigger

import math
import numpy as np

# Defined Macros
RECOVERY_ATTEMPTS = 3
RECOVERY_PERIOD = 10
MIN_RADIUS = 8
MAX_RADIU = 12
RADIAL_STEP = 1
YAW_STEP = 15

class RecoveryHelper(Node):
    def __init__(self):
        super().__init__('RecoveryHelper')

        # Class variables ______________________________________________________
        self.navigator = BasicNavigator()
        self.latest_odom = None
        self.latest_costmap = None
        self.current_goal = None

        # Create timer _________________________________________________________
        #self.timer = self.create_timer(5.0, self.timer_tick)

        # Create action clients ________________________________________________
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')

        self.sub_odom = self.create_subscription(
                Odometry,
                '/odom/',
                self.odom_callback,
                10
                )

        self.sub_costmap = self.create_subscription(
                local_costmap,
                'local_costmap/costmap',
                self.costmap_callback,
                10
                )

        self.recovery_srv = self.create_service(
                RecoveryTrigger,
                'call_recovery',
                self.recovery_callback
                )

        self.navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        
        self.get_logger().info("RecoveryHelper node intialised")

    # Callback functions _______________________________________________________
    def 

    def odom_callback(self, msg: Odometry):
        if msg is None:
            self.get_logger().warn("Odom data was not received")
            return
        self.latest_odom = msg

    def costmap_callback(self, msg: Costmap):
        if msg is None:
            self.get_logger().warn("Local costmap data was not received")
            return
        self.latest_costmap = msg

    def recovery_callback(self, request, response):
        
        # Trigger a manual failure recovery
        outcome = self.failure_recovery()
        if outcome:
            self.get_logger().info("Successfully recovered robot")
        else
            self.get_logger().info("Failed to recover robot")
        response.success = outcome
        return response

    # Node Methods _____________________________________________________________
    def get_robot_pose(self) -> tuple[float, float, float]:
        if self.latest_odom is None:
            self.get_logger().warn("Could not gather pose from odom")
            return None
        px = self.latest_odom.pose.pose.position.x
        py = self.latest_odom.pose.pose.position.y
        pw = self.latest_odom.pose.pose.orientation.w
        return (px, py, pw)

    def get_local_costmap(self):
        if self.latest_costmap is None:
            self.get_logger().warn("Failed to get local costmap data")
            return None
        return self.latest_costmap

    def check_nav_active(self) -> bool:
        self.navigator.get_logger().info("Waiting for Navigation action server")
        ready = false

        for check in range(5):
            ready = self.navigator.nav_to_pose_client.wait_for_server(timeout_sec = 5.0)
            if ready:
                return True
            self.navigator.get_logger().info("Navigation action server not avialable, waiting...")

        self.get_logger().error("Navigator server was never avaialable")
        return False

    def failure_recovery(self) -> bool:

        self.get_logger().warn("### Failure recovery called ###")
        recovery_successs = False

        for attempt in range(RECOVERY_ATTEMPTS):
            self.get_logger().info(f"Attempt no. {attempt}")

            # Get current data
            robot = self.get_robot_pose()
            costmap = self.get_local_costmap()

            if (pose or costmap is None):
                self.get_logger().error(f"Attempt {attempt} failed due to no data")
                continue
            else:
                rx, ry, rw = pose[0], pose[1], pose[2]
                info = costmap.info
                grid = np.array(costmap.data, dtype=np.int8)
                grid = grid.reshape((info.height, info.width))

            gx, gy = self.get_recovery_pose(px, py, grid, info)

            spin_success = self.manual_spin(px, py, pw, goal)
            if not spin_success:
                self.get_logger().warn("Spin failed")
                continue

            drive_success = self.manual_drive(px, py, pw, goal)
            if not drive_success:
                self.get_logger().warn("Drive failed")
                continue

            return True
        self.get_logger().warn(f"Recovery failed after {RECOVERY_ATTEMPTS}")
        return False
    
    def manual_spin(self, px, py, pw, goal) -> bool:

        dx = abs(px) - abs(goal[0])
        dy = abs(py) - abs(goal[1])

        goal_yaw = atan2(dx, dy)
        delta_yaw = math.atan(np.sin(goal_yaw - pw), np.cos(goal_yaw - pw))

        if not self.spin_client.server_is_ready():
            self.get_logger().warn("spin server is not ready")
            return False

        goal = Spin.Goal()
        goal.target_yaw = float(delta_yaw)
        goal.target_allowance = Duration(sec=RECOVERY_PERIOD)
    

        response = self.spin_client.send_goal(goal)
        self.get_logger().info("Spin action sent for {delta_yaw:.5f}")

        return (response is not None and 
                response.status == GoalStatus.STATUS_SUCCEEDED)

    def manual_drive(self, px, py, goal) -> bool:
        
        if not self.drive_client.wait_for_server(timeout_sec=RECOVERY_PERIOD):
            self.get_logger().warn("Drive_on_heading server not ready")
            return False

        goal = DriveOnHeading.Goal()
        goal.target.x = float(math.dist((px,py), goal))
        # REVERSE OR FORWARD CHECKING???
        goal.speed = DRIVE_SPEED
        goal.time_allowance = Duration(sec=RECOVERY_PERIOD)

        response = self.drive_client.send_goal(goal)

        return (response is not None and 
                response.status == GoalStatus.STATUS_SUCCEEDED)
    
    def world_to_grid(self, wx, wy, info) -> tuple[int, int]:
        ox, oy = info.origin.position.x, info.origin.position.y

        gx = math.floor((wx - ox) / info.resolution)
        gy = math.floor((wy - oy) / info.resolution)

        if (0 <= gx <= info.width) and (0 <= gy <= info.height):
            return gx, gy
        return None

    def grid_to_world(self, gx, gy, info) -> tuple[float, float]:
        
        wx = info.origin.position.x + (gx + 0.05) * info.resolution
        wy = info.origin.position.y + (gy + 0.05) * info.resolution

        return float(wx, wy)

    def get_recovery_pose(self, pose, grid, info) -> tuple[float, float]:
        
        gx, gy = self.world_to_grid(pose[0], pose[1], info)

        # Search bearings from 0 to 2pi
        bearings = np.linspace(0, 2*np.pi, (360 / YAW_STEP), endpoint=false)
        ideal_yaw = 0

        # Search across bearings
        for yaw in bearings:
            xs = gx + MIN_RADIUS * np.cos(yaw)
            ys = gy + MIN_RADIUS * np.sin(yaw)

            cost = self.get_cost((xs, ys), grid, True)

            ideal_yaw = (ideal_yaw + (yaw * cost)) / 2

        # March radius along ideal bearing
        ideal_radius = MIN_RADIUS
        for radius in range(MIN_RADIUS, MAX_RADIUS, RADIAL_STEP):
            xs = gx + radius * np.cos(ideal_yaw)
            yx = gy + radius * np.sin(ideal_yaw)

            cost = self.get_cost((xs, ys), grid, True)

            ideal_radius = (ideal_radius + (radius * cost)) / 2

        recovery_pose = ((gx + ideal_radius * np.cos(ideal_yaw)),
                            gy + ideal_radius * np.sin(ideal_yaw))
        return recovery_pose

    def get_cost(self, pose, grid, invert:bool) -> int:
        cost = int(grid[pose[0], pose[1]])
        # Cost determination
        match cost:
            case -1:    #   UKNWOWN Cost
                cost = 100
            case 99:    # INSCRIBED Cost
                cost = 100
            case 100:   # LETHAL Cost
                cost = 100
            case _:     # Cost is 0 - 98
                cost = cost

        # Invert cost if flagged
        if invert:
            return 100 - cost
        return cost

    def destroy_navigator(self) -> None:
        self.navigator.destroy_node()

