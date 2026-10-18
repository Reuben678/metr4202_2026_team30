from dataclasses import dataclass
from math import hypot
import math
from typing import List, Optional, Tuple
import json

from nav2_simple_commander.robot_navigator import BasicNavigator

import networkx as nx
from metr4202_interfaces.srv import GetFrontiers
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA

Position = Tuple[float, float]

FRONTIER_COLS = 5           # Expected columns in frontier msg

MERGE_TOLERANCE = 0.8       # Allowable tolerance between frontier nodes [m]
VISITED_TOLERANCE = 0.5     # Suppress frontiers near visited waypoints [m]

EDGE_SEARCH_RADIUS = 3 * VISITED_TOLERANCE    # [m] only plan to visited nodes within this straight-line distance
MAX_EDGE_COUNT = 3

UNASSIGNED_ID = -1          # Place holder

NODE_MARKER_SIZE = 0.05     # [m] one map cell              # Node marker size
VISITED_COLOUR = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)      # Visited node marker RGB
UNVISITED_COLOUR = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)    # Unvisited node marker RGB

@dataclass
class Frontier:
    frontier_id: int
    label: int
    size: int
    x: float
    y: float
    utility: float
    visited: bool = False

    @property
    def position(self) -> Position:
        return self.x, self.y

class GraphManager:

    START_NODE_ID = 0

    def __init__(self, node: Node) -> None:
        self.node = node

        # Create navigation tools
        self.path_navigator = BasicNavigator(node_name="frontier_path_navigator")
        self.path_navigator.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # RViz display of the graph nodes
        self.marker_pub = self.node.create_publisher(Marker, "graph_nodes", 10)

        # Service client used to request the latest output from Frontier Search.
        self.frontier_client = self.node.create_client(GetFrontiers, "get_frontiers")
 
        # Stores the current asynchronous frontier request while it finishes.
        self.frontier_future = None
 
        # Persistent graph, kept for the whole search.
        # Each node stores its Frontier object under the "frontier" attribute,
        # each edge stores the path distance between its nodes under "weight".
        self.graph = nx.Graph()
 
        # Node the robot is currently at
        self.current_node_id: Optional[int] = None
 
        # Increasing ID handed to each new frontier node
        self.next_node_id = self.START_NODE_ID + 1

        # ArUco Integration
        # Store detected ArUco landmarks, keyed by marker ID.
        self.aruco_nodes = {}

        # Subscribe to ArUco markers
        self.aruco_subscription = self.node.create_subscription(
            String,
            "/targets/aruco",
            self.aruco_callback,
            10,
        )
 
    def initialise(self, robot_position: Position):
        start = Frontier(
            frontier_id = self.START_NODE_ID,
            label = UNASSIGNED_ID,
            size = 0,
            x = robot_position[0],
            y = robot_position[1],
            utility = 0.0,
            visited = True,
        )
        self.graph.add_node(self.START_NODE_ID, frontier=start)
        self.current_node_id = self.START_NODE_ID
        self.publish_markers()

    def get_node(self, node_id: int) -> Frontier:
        return self.graph.nodes[node_id]["frontier"]

    def current_node(self) -> Frontier:
        return self.get_node(self.current_node_id)
    
    def has_unvisited(self) -> bool:
        return any(not self.get_node(node_id).visited for node_id in self.graph.nodes)
    
    def publish_markers(self) -> None:
        """Publish every graph node position as RED(unvisited) or GREEN(visited) for RViz"""

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "graph_nodes"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = NODE_MARKER_SIZE
        marker.scale.y = NODE_MARKER_SIZE
        marker.scale.z = 0.02      # Flat, sitting just above the map
        marker.color.a = 1.0

        for node_id in list(self.graph.nodes):
            node = self.get_node(node_id)
            marker.points.append(Point(x=node.x, y=node.y, z=0.01))
            if node.visited:
                marker.colors.append(VISITED_COLOUR)
            else:
                marker.colors.append(UNVISITED_COLOUR)

        self.marker_pub.publish(marker)

    def remove_stale_frontiers(self, latest_frontiers: List[Frontier]) -> None:
        """Remove unvisited/outdated graph nodes from the latest frontier results."""

        stale_node_ids = []

        for node_id in list(self.graph.nodes):

            node = self.get_node(node_id)
            # Retain visited nodes for back traversal
            if node.visited:
                node = self.get_node(node_id)
                self.node.get_logger().info(
                    f"Removing stale frontier {node_id} "
                    f"at ({node.x:.2f}, {node.y:.2f})"
                )
                continue

            # Check whether the node corresponds to a current frontier
            still_valid = any(hypot(node.x - frontier.x, node.y - frontier.y) <= MERGE_TOLERANCE for frontier in latest_frontiers)

            if not still_valid:
                stale_node_ids.append(node_id)

        for node_id in stale_node_ids:
            self.graph.remove_node(node_id)

        if stale_node_ids:
            self.node.get_logger().info(f"Removed {len(stale_node_ids)} outdated frontier nodes.")

    def request_frontiers(self) -> bool:
        if self.frontier_future is not None:
            return True
        
        if not self.frontier_client.service_is_ready():
            self.node.get_logger().warning("GetFrontiers service not available")
            return False
        
        request = GetFrontiers.Request()
        request.end_search = False
        self.frontier_future = self.frontier_client.call_async(request)
        self.node.get_logger().info("Requested current frontiers")
        return True

    def process_frontier_response(self) -> Optional[List[int]]:
        if self.frontier_future is None or not self.frontier_future.done():
            return None
        
        future = self.frontier_future
        self.frontier_future = None

        try:
            response = future.result()
        except Exception as error:
            raise RuntimeError(f"GetFrontiers request failed: {error}")
        
        frontiers = self.parse_frontier_response(response)
        self.node.get_logger().info(
            f"Frontier Search returned {len(frontiers)} candidates:"
        )

        for frontier in frontiers:
            self.node.get_logger().info(
                f"Candidate: x={frontier.x:.2f}, "
                f"y={frontier.y:.2f}, "
                f"utility={frontier.utility:.3f}"
            )
            
############# Added to remove outdated unvisited frontiers  ################
        self.remove_stale_frontiers(frontiers)

        # Add or update valid frontiers
        node_ids = []

        for frontier in frontiers:
            node_id = self.add_frontier(frontier)
            if node_id is not None:
                node_ids.append(node_id)

        self.node.get_logger().info(
            f"Graph status: {self.graph.number_of_nodes()} total nodes, "
            f"{sum(not self.get_node(n).visited for n in self.graph.nodes)} "
            f"unvisited frontiers remaining."
        )
        
        self.publish_markers()
        return node_ids
    
    def send_end_search(self):

        if not self.frontier_client.service_is_ready():
            self.node.get_logger().warning("GetFrontiers service is not available")
            return None
        
        request = GetFrontiers.Request()
        request.end_search = True
        return self.frontier_client.call_async(request)

    def parse_frontier_response(self, response) -> List[Frontier]:

        if response is None:
            raise RuntimeError("GetFrontiers returned no response")

        if not response.success:
            raise RuntimeError("FrontierSearch reported detection failure")

        if response.empty:
            return []
        
        message = response.frontiers
        return self.parse_frontiers(list(message.data), int(message.rows),
                                    int(message.cols))
    
    def parse_frontiers(self, data: List[float], rows: int, columns: int) -> List[Frontier]:

        if rows < 0:
            raise ValueError("Frontier rows is negative")
        
        if columns != FRONTIER_COLS:
            raise ValueError(f"Expected {FRONTIER_COLS} values per frontier")

        expected_length = rows * columns
 
        if len(data) != expected_length:
            raise ValueError(f"Expected {expected_length} frontier values, but received {len(data)}.")
 
        frontiers: List[Frontier] = []
 
        for row_index in range(rows):
            start = row_index * columns
            row = data[start:start + columns]
            frontiers.append(
                Frontier(
                    frontier_id=UNASSIGNED_ID,
                    label=int(row[0]),
                    size=int(row[1]),
                    x=float(row[2]),
                    y=float(row[3]),
                    utility=float(row[4]),
                )
            )
        return frontiers
    
    def find_proxy_node(self, position: Position, visited: bool) -> Optional[int]:
 
        best_id = None
        tolerance = (VISITED_TOLERANCE if visited else MERGE_TOLERANCE )
        min_proximity = tolerance
 
        for node_id in self.graph.nodes:
            node = self.get_node(node_id)
 
            if node.visited != visited:
                continue
 
            distance = hypot(node.x - position[0], node.y - position[1])
 
            if distance <= min_proximity:
                best_id = node_id
                min_proximity = distance
 
        return best_id

    def make_pose(self, position: Position, yaw=0.0) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.path_navigator.get_clock().now().to_msg()
        p.pose.position.x = float(position[0])
        p.pose.position.y = float(position[1])
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def path_length(self, path) -> float:
        distance = 0.0
        for a, b in zip(path.poses[:-1], path.poses[1:]):
            distance += math.hypot(
                b.pose.position.x - a.pose.position.x,
                b.pose.position.y - a.pose.position.y
            )
        return distance

    """
    Compute the path distance to (wx, wy) from Nav2
    """
    def compute_path_distance(self, goal, start) -> float:
            """
            Ask the Nav2 planner for a path from the robot to (wx, wy) in the map
            frame and return its length [m], or None if no path was found
            """
            # Create PoseStamped from Position to compute path distance
            goal_pose = self.make_pose(goal)
            start_pose = self.make_pose(start)

            # use_start=True -> the planner starts from start point given
            path = self.path_navigator.getPath(start_pose, goal_pose, use_start=True)
    
            if path is None or len(path.poses) == 0:
                return None

            return self.path_length(path)

    """
    Compute which node is most ideal to connect to current position, using
    lowest path cost computed, returning node_id and path cost
    """
    def get_node_edge(self, frontier_id: int, position: Position):
        
        candidates = []
        for node_id in self.graph.nodes:
            if node_id == frontier_id:
                continue
            node = self.get_node(node_id)
            if not node.visited:    # Only visited nodes accepted
                continue
            euclid = hypot(node.x - position[0], node.y - position[1])
            if euclid <= EDGE_SEARCH_RADIUS:
                distance = self.compute_path_distance( position,(node.x, node.y))
                if distance is None:
                    continue
                candidates.append((distance, node_id))
            
        if not candidates:
            self.node.get_logger().warning("NO LOCAL CANDIDATES FOR EDGE")
            return None

        candidates.sort()
        best_edges = candidates[:MAX_EDGE_COUNT]    # Only keep the top N edges
        
        return best_edges

    def add_frontier(self, frontier: Frontier) -> Optional[int]:
    
            # Already been here: discard, otherwise the robot would revisit it.
            visited_id = self.find_proxy_node(
                frontier.position,
                visited=True
            )

            if visited_id is not None:
                self.node.get_logger().info(
                    f"Rejecting frontier at ({frontier.x:.2f}, {frontier.y:.2f}): "
                    f"within {VISITED_TOLERANCE:.2f} m of visited node {visited_id}"
                )
                return None
    
            existing_id = self.find_proxy_node(frontier.position, visited=False)
    
            if existing_id is not None:
                # Merge: keep the existing ID, take the latest data, and re-attach the
                # node to the current node since it is now in view from here.
                node = self.get_node(existing_id)
                node.label = frontier.label
                node.size = frontier.size
                node.x = frontier.x
                node.y = frontier.y
                node.utility = frontier.utility
    
                self.graph.remove_edges_from(list(self.graph.edges(existing_id)))
                node_id = existing_id
                #debugging
                self.node.get_logger().info(
                    f"Merging frontier at ({frontier.x:.2f}, {frontier.y:.2f}) "
                    f"with unvisited node {existing_id}"
                )
            else:
                node_id = self.next_node_id
                self.next_node_id += 1
                frontier.frontier_id = node_id
                self.graph.add_node(node_id, frontier=frontier)
    
            # Add edge from nearest node to frontier
            ### TO BE ADDED ####
            edges = self.get_node_edge(frontier.frontier_id, frontier.position)
            if edges is None:
                self.node.get_logger().warning("Could not find suitable edge")
                self.graph.remove_node(node_id)
                return None
            
            for edge in edges:
                edge_dist, local_node_id = edge[0], edge[1]
                self.graph.add_edge(local_node_id, node_id, weight=edge_dist)
            return node_id

    def mark_reached(self, node_id: int) -> None:
        """Mark a node as visited and make it the robot's current node."""
 
        self.get_node(node_id).visited = True
        self.current_node_id = node_id
        self.publish_markers()

    def remove_node(self, node_id: int) -> bool:

        if node_id not in self.graph:
            return False

        if self.get_node(node_id).visited:
            self.node.get_logger().warning(f"Node {node_id} is visited and cannot be removed")
            return False

        if node_id not in self.graph:
            return False
        
        self.graph.remove_node(node_id)
        self.publish_markers()
        self.node.get_logger().info(f"Removed unreachable node {node_id}")
        return True
    
    # Goal selection ____________________________________________________________
    def select_goal(self) -> Optional[Frontier]:
 
        candidates = [
            self.get_node(node_id)
            for node_id in self.graph.neighbors(self.current_node_id)
            if not self.get_node(node_id).visited
        ]
 
        if not candidates:
            return None
 
        return max(candidates, key=lambda frontier: frontier.utility)    

    def plan_traversal(self) -> Optional[List[Frontier]]:
        """Find the shortest route to an unvisited frontier through visited nodes."""

        # Only allow visited nodes for intermediate traversal.
        visited_nodes = [node_id for node_id in self.graph.nodes 
                         if self.get_node(node_id).visited]

        # Construct a temporary graph containing visited nodes.
        visited_graph = self.graph.subgraph(visited_nodes)

        # Find shortest distances through previously visited nodes.
        distances, paths = nx.single_source_dijkstra(visited_graph, self.current_node_id, weight="weight")

        best_target = None
        best_path = None
        best_distance = float("inf")

        # Find unvisited frontiers connected to the visited graph.
        for node_id in distances:

            for neighbor_id in self.graph.neighbors(node_id):

                if self.get_node(neighbor_id).visited:
                    continue

                total_distance = (distances[node_id] + self.graph[node_id][neighbor_id]["weight"])

                if total_distance < best_distance:
                    best_distance = total_distance
                    best_target = neighbor_id
                    best_path = paths[node_id] + [neighbor_id]

        if best_target is None:
            return None

        # Exclude the robot's current node.
        return [self.get_node(node_id) for node_id in best_path[1:]]

    def aruco_callback(self, msg: String) -> None:
        """Create a landmark entry when a new ArUco ID is detected."""

        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as error:
            self.node.get_logger().warning(
                f"Invalid marker JSON: {error}"
            )
            return

        if not isinstance(data, dict):
            return

        try:
            marker_id = int(data["id"])
            x = float(data["pos_x"])
            y = float(data["pos_y"])

            timestamp = data["time"]
            time_sec = int(timestamp["sec"])
            time_nanosec = int(timestamp["nanosec"])

            if not all(math.isfinite(v) for v in (x, y)):
                return

        except (KeyError, TypeError, ValueError, OverflowError):
            self.node.get_logger().warning(
                "Marker JSON is missing valid id, pos_x, pos_y, or time fields."
            )
            return

        # Only create an entry the first time this ID is detected.
        if marker_id in self.aruco_nodes:
            return

        self.aruco_nodes[marker_id] = {
            "id": marker_id,
            "x": x,
            "y": y,
            "time": {
                "sec": time_sec,
                "nanosec": time_nanosec,
            },
        }

        self.node.get_logger().info(
            f"New ArUco marker {marker_id} at "
            f"x={x:.2f}, y={y:.2f}, "
            f"time={time_sec}.{time_nanosec:09d}"
        )

    def get_marker_node(self, marker_id: int):
        """Get a stored landmark by ID, or None if it is unknown."""
        return self.aruco_nodes.get(marker_id)
