from dataclasses import dataclass
from math import hypot
from typing import List, Optional, Tuple

import networkx as nx
from metr4202_interfaces.srv import GetFrontiers
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

NODE_MARKER_SIZE = 0.05     # [m] one map cell

Position = Tuple[float, float]

FRONTIER_COLS = 6       # Expected columns in frontier msg

MERGE_TOLERANCE = 1.0   # Allowable tolerance between frontier nodes [m]

UNASSIGNED_ID = -1      # Place holder

@dataclass
class Frontier:
    frontier_id: int
    label: int
    size: int
    x: float
    y: float
    path_distance: float
    utility: float
    visited: bool = False

    @property
    def position(self) -> Position:
        return self.x, self.y

class GraphManager:

    START_NODE_ID = 0

    def __init__(self, node: Node) -> None:
        self.node = node
 
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
 
    def initialise(self, robot_position: Position):
        start = Frontier(
            frontier_id = self.START_NODE_ID,
            label = UNASSIGNED_ID,
            size = 0,
            x = robot_position[0],
            y = robot_position[1],
            path_distance = 0.0,
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
        """Publish every graph node position as a yellow cell for RViz."""

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

        for node_id in self.graph.nodes:
            node = self.get_node(node_id)
            if node.visited:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

            marker.points.append(Point(x=node.x, y=node.y, z=0.01))

        self.marker_pub.publish(marker)

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
        
        frontiers=self.parse_frontier_response(response)

        node_ids = []
        for frontier in frontiers:
            node_id = self.add_frontier(frontier)
            if node_id is not None:
                node_ids.append(node_id)

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
                    path_distance=float(row[4]),
                    utility=float(row[5]),
                )
            )
        return frontiers
    
    def find_nearby_node(self, position: Position, visited: bool) -> Optional[int]:
 
        best_id = None
        min_proximity = MERGE_TOLERANCE
 
        for node_id in self.graph.nodes:
            node = self.get_node(node_id)
 
            if node.visited != visited:
                continue
 
            distance = hypot(node.x - position[0], node.y - position[1])
 
            if distance <= min_proximity:
                best_id = node_id
                min_proximity = distance
 
        return best_id

    def add_frontier(self, frontier: Frontier) -> Optional[int]:
    
            # Already been here: discard, otherwise the robot would revisit it.
            if self.find_nearby_node(frontier.position, visited=True) is not None:
                return None
    
            existing_id = self.find_nearby_node(frontier.position, visited=False)
    
            if existing_id is not None:
                # Merge: keep the existing ID, take the latest data, and re-attach the
                # node to the current node since it is now in view from here.
                node = self.get_node(existing_id)
                node.label = frontier.label
                node.size = frontier.size
                node.x = frontier.x
                node.y = frontier.y
                node.path_distance = frontier.path_distance
                node.utility = frontier.utility
    
                self.graph.remove_edges_from(list(self.graph.edges(existing_id)))
                node_id = existing_id
            else:
                node_id = self.next_node_id
                self.next_node_id += 1
                frontier.frontier_id = node_id
                self.graph.add_node(node_id, frontier=frontier)
    
            self.graph.add_edge(self.current_node_id, node_id, weight=frontier.path_distance)
            return node_id

    def mark_reached(self, node_id: int) -> None:
        """Mark a node as visited and make it the robot's current node."""
 
        self.get_node(node_id).visited = True
        self.current_node_id = node_id
        self.publish_markers()

    def remove_node(self, node_id: int) -> bool:
        
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
 
        distances, paths = nx.single_source_dijkstra(self.graph, self.current_node_id, weight="weight")
 
        candidates = [node_id for node_id in distances if not self.get_node(node_id).visited]
 
        if not candidates:
            return None
 
        target_id = min(candidates, key=lambda node_id: distances[node_id])
 
        # Drop the first entry, which is the current node.
        return [self.get_node(node_id) for node_id in paths[target_id][1:]]





