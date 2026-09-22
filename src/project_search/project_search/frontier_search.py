"""
frontier_search.py
Author: Mitchell Crawford (s4584081)
METR4202, Sem2, 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from metr4202_interfaces.msg import FrontierArray
from metr4202_interfaces.srv import UtilWeights
from metr4202_interfaces.srv import GetFrontiers

import numpy as np
from scipy import ndimage


FREE, UNKNOWN, OCCUPIED = 0, -1, 100    # Defined macros for trinary cell values

MIN_FRONTIER_SIZE = 5   # Min size of frontier allowable 
MIN_FRONTIER_DIST = 3

MAP_FRAME = "map"
ROBOT_FRAME = "base_link"

BT_TRIGGER_NODE = "NavigateRecovery"
BT_TRIGGER_STATUS = ("SUCCESS", "FAILURE")

class FrontierSearch(Node):
    def __init__(self):
        super().__init__('FrontierSearch')

        # Class variables
        self.latest_map = None
        self.latest_odom = None
        self.w_info = 1.0
        self.w_cost = 1.0
        self.weight_future = None

        # Create subscriptions
        # ___________________________________________________________ 
        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)
    
        self.occ_grid_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create publishers
        # ___________________________________________________________
        self.frontier_pub = self.create_publisher(
            PoseStamped,
            'frontiers',
            10
        )

        # Create Interfaces
        # ___________________________________________________________

        self.weight_client = self.create_client(
            UtilWeights,
            'util_weights_service'
            )
        
        for attempt in range(10):
            if self.weight_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('UtilWeights service found')
                break
            if (attempt == 9):
                self.get_logger().error('UtilWeights not available')
                raise RuntimeError('UtilWeights service never became available')

        # Send an initial weights request
        self.send_request()

        # Initially had FrontierData here, but not defined. Changed to GetFrontiers
        #MST client is also looking for get_frontiers
        self.frontier_srv = self.create_service(
            GetFrontiers,
            'get_frontiers',
            self.frontier_resp_callback
        )
       
       # Initialisation complete
        self.get_logger().info('FrontierSearch node initialised')

        # nav goals handled in exploration manager
        # # Start BehaviorTreeLog with simple goal request to current pose
        # goal = PoseStamped()
        # goal.header.frame_id = 'map'
        # goal.header.stamp = self.get_clock().now().to_msg()
        # goal.pose.position.x, goal.pose.position.y = self.get_robot_pose()
        # goal.pose.position.z = 0.0
        # goal.pose.orientation.w = 1.0      

        # # LET THE NAVIGATION BEGIN!!
        # self.goal_pub.publish(goal)  

    # Callback functions
    # _______________________________________________________________
    
    def frontier_resp_callback(self, request, response):
        if self.latest_map is None:
            response.success = False
            return response
        # Update our map information
        info = self.latest_map.info

        #class methods need self, fixed the clusters naming error below 
        frontier_mask = self.process_grid()
        if frontier_mask is None:
            response.success = False
            return response 
        
        clusters = self.cluster_frontiers(frontier_mask)

        # Check if clusters were found
        if not clusters:
            self.get_logger().info("No frontiers were found within the grid")
            return
        self.get_logger().info(f"{len(clusters)} frontiers(s) found")

        response.frontiers = self.package_frontiers(clusters, 
                                info.resolution,
                                info.origin.position
                            )
        response.success = True
        return response        

    """ Has troubles with this function because the client and future names were different,
        and didn't check whether self.weight_client was ready. It was changed to an async request, below."""
    # def weights_request(self, timeout = 5.0):
        
    #     if not self.weight_client(timout_sec=timeout):
    #         return None
    #     # Send a call to the UtilWeights server
    #     future = self.weights_client.call_async(UtilWeights.Request())
    #     # Spin this call until we get a response (or timeout)
    #     rclpy.spin_until_future_complete(self.future, timeout_sec = timeout)
    #     response = future.result()
    #     # Check if we got a response
    #     if response is not None:
    #         # Process the weights from this response
    #         self.w_cost = response.w_cost
    #         self.w_info = response.w_info
    #         return
    #     # Response was empty
    #     self.get_logger().warn("UtilWeights returned no response")
    #     return None

    def send_request(self) -> None:
        """Request the latest utility weights asynchronously."""

        if not self.weight_client.service_is_ready():
            self.get_logger().warning("UtilWeights service is not ready.")
            return

        if (hasattr(self, "weight_future") and self.weight_future is not None and not self.weight_future.done()):
            # A request is already in progress.
            return

        request = UtilWeights.Request()

        self.weight_future = (self.weight_client.call_async(request))

        self.weight_future.add_done_callback(self.weights_response_callback)

    def weights_response_callback(self, future) -> None:
        """Store utility weights returned by the service."""

        try:
            response = future.result()

        except Exception as error:
            self.get_logger().error(f"UtilWeights request failed: {error}")
            self.weight_future = None
            return

        if response is None:
            self.get_logger().warning("UtilWeights returned no response.")
            self.weight_future = None
            return

        self.w_cost = response.w_cost
        self.w_info = response.w_info

        self.get_logger().info(f"Updated utility weights: " f"w_info={self.w_info:.3f}, "f"w_cost={self.w_cost:.3f}")
        self.weight_future = None

    # Callback to read from BehaviorTreeLog topic
    def bt_log_callback(self, msg:BehaviorTreeLog):
        for event in msg.event_log:
            if event.node_name == BT_TRIGGER_NODE and \
                    event.previous_status == "RUNNING" and \
                    event.current_status in BT_TRIGGER_STATUS:
                self.send_request() # Request new weights before frontier search
                return
        return

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
    
    # Use callback to get any new messages for OccupancyGrid
    def map_callback(self, msg:OccupancyGrid):
        self.latest_map = msg

    # Node methods
    # _______________________________________________________________
    """
    Find cells which form a frontier (FREE cells bordering UNKNOWN cells)
    """
    @staticmethod
    def find_frontier_mask(grid):
        free_mask = grid == FREE
        unknown_mask = grid == UNKNOWN

        # Copy contents of unknown_mask, with same data type and shape
        neighbour_unknown = np.zeros_like(unknown_mask)
        """
        Take OR opertation of neighbour grid with mask of unknown cells, shifting
        the mask of unknown cells to determine if a given cell has an neighbouring
        cell that is marked as UNKNOWN
        """ 
        neighbour_unknown[1:, :] |= unknown_mask[:-1, :]
        neighbour_unknown[:-1, :] |= unknown_mask[1:, :]
        neighbour_unknown[:, 1:] |= unknown_mask[:, :-1]
        neighbour_unknown[:, :-1] |= unknown_mask[:, 1:]
        # AND mask of free cells and cells with UNKNOWN neighbours
        return free_mask & neighbour_unknown

    """
    Cluster cells of frontiers
    """
    @staticmethod
    def cluster_frontiers(frontier_mask):
        structure = np.ones((3,3), dtype=int) # 8-connectivity
        clusters = []
        labeled, num = ndimage.label(frontier_mask, structure=structure)

        for label_id in range(1, num + 1):
            ys, xs = np.where(labeled == label_id)

            size = len(xs)
            centroid_px = (float(np.mean(xs)), float(np.mean(ys)))

            centroid_dist = np.hypot(centroid_px[0], centroid_px[1])

            if (size < MIN_FRONTIER_SIZE or centroid_dist < MIN_FRONTIER_DIST):
                continue

            clusters.append({"label": label_id, "size": size, "centroid_px": centroid_px})

        return clusters

    """
    Convert grid cell indexs to world coords
    """
    def grid_to_world(gx, gy, resolution, origin_x, origin_y):
        world_x = origin_x + (gx + 0.5) * resolution
        world_y = origin_y + (gy + 0.5) * resolution
        return world_x, world_y

    """
    Score + rank by combined utility
    """
    def score_frontiers(clusters, resolution, origin_x, origin_y, robot_pose):
        rx, ry = robot_pose
        for c in clusters:
            px, py = c["centroid_px"]
            wx, wy = grid_to_world(px, py, resolution, origin_x, origin_y)
            c["centroid"] = (wx, wy)
            c["distance"] = float(np.hypot(wx - rx, wy - ry))
            c["utility"] = self.w_info * c["size"] - self.w_cost * c["distance"]
        
        return sorted(clusters, key=lambda c: c["utility"], reverse=True)

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
    Pacakge frontiers for publishing
    """
    def package_frontiers(self, clusters, resolution, origin):
        rows = [
            (c["label"], c["size"],
            origin.x + (c["centroid_px"][0] + 0.5) * resolution,
            origin.y + (c["centroid_px"][1] + 0.5) * resolution)
            for c in clusters
        ]
        arr = np.array(rows, dtype=np.float32).reshape(-1, 4)

        msg = FrontierArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.rows, msg.cols = arr.shape
        msg.data = arr.ravel().tolist()
        return msg

    """
    Publish the packaged frontiers data
    """
    def publish_package(self, package):
        msg = FrontierArray()
        msg.header.stamp = self.get_clock().now.to_msg()
        msg.header.frame_id = "map"
        msg.rows, msg.cols = package.shape
        msg.data = package.ravel().tolist()
        self.frontier_pub.publish(msg)

    """
    Process all grid data and publish
    """
    def process_grid(self):
        # 1) Update map information
        if self.latest_map is None:
            self.get_logger().debug("No new map recieved")
            return
        
        # 2) Update robot pose information
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
        
        # 3) Break down occupancy grid message
        # Get msg from latest_map variable
        msg = self.latest_map
        # Process info from message
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Process message data from 1D array to 2D array (grid)
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # 4) Determine mask grid for frontiers
        frontier_mask = self.find_frontier_mask(grid)
        return frontier_mask

def main():
    rclpy.init()

    # Create frontier_search node
    frontier_search = FrontierSearch()

    try:
        # Execute node
        rclpy.spin(frontier_search)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop node spinning (destroy and shutdown)
        frontier_search.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()