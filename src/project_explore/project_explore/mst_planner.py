from dataclasses import dataclass
from math import hypot, isfinite
from typing import Callable, List, Optional, Tuple

import networkx as nx
from metr4202_interfaces.srv import GetFrontiers
from rclpy.node import Node


Position = Tuple[float, float]
TravelCostFunction = Callable[[Position, Position], float]


@dataclass(frozen=True)
class Frontier:
    """Store a frontier ID and its map position."""

    frontier_id: int
    x: float
    y: float


@dataclass(frozen=True)
class Edge:
    """Store a weighted connection between two graph nodes."""

    node_a: int
    node_b: int
    cost: float


class MSTPlanner:
    """Request frontier data and generate an ordered MST plan."""

    # Use -1 for the robot so it cannot be confused with a frontier ID.
    ROBOT_NODE_ID = -1

    def __init__(self, node: Node, travel_cost_function: Optional[TravelCostFunction] = None) -> None:
        self.node = node

        # Euclidean distance is used unless another travel cost method is supplied.
        self.travel_cost_function = travel_cost_function or self.euclidean_travel_cost

        # Service client used to request the latest output from Frontier Search.
        self.frontier_client = self.node.create_client(GetFrontiers, "get_frontiers")

    @staticmethod
    def euclidean_travel_cost(start: Position, goal: Position) -> float:
        """Calculate the straight-line distance between two positions."""

        return hypot(goal[0] - start[0], goal[1] - start[1])

    @staticmethod
    def parse_frontiers(data: List[float], rows: int, columns: int) -> List[Frontier]:
        """Convert Frontier Search rows into Frontier objects."""

        # Each Frontier Search row is [label, size, world_x, world_y].
        # Frontier size is supplied but is not currently needed by the MST Planner.
        if rows < 0:
            raise ValueError("The number of frontier rows cannot be negative.")

        if columns != 4:
            raise ValueError("Expected frontier rows in the format [label, size, world_x, world_y].")

        expected_length = rows * columns

        if len(data) != expected_length:
            raise ValueError(f"Expected {expected_length} frontier values, but received {len(data)}.")

        frontiers: List[Frontier] = []

        for row_index in range(rows):
            start = row_index * columns
            row = data[start:start + columns]
            frontiers.append(
                Frontier(
                    frontier_id=int(row[0]),
                    x=float(row[2]),
                    y=float(row[3]),
                )
            )

        return frontiers

    def parse_frontier_response(self, response) -> List[Frontier]:
        """Check the service response and extract its Frontier objects."""

        if response is None:
            raise RuntimeError("GetFrontiers returned no response.")

        if not response.success:
            raise RuntimeError("Frontier Search reported that detection failed.")

        message = response.frontiers
        return self.parse_frontiers(list(message.data), int(message.rows), int(message.cols))

    def receive_frontiers(self):
        """Request the latest frontiers without blocking the node."""

        if not self.frontier_client.service_is_ready():
            self.node.get_logger().warning("GetFrontiers service is not available.")
            return None

        request = GetFrontiers.Request()
        return self.frontier_client.call_async(request)

    def construct_graph(self, frontiers: List[Frontier], robot_position: Position) -> nx.Graph:
        """Build a complete weighted graph from the robot and frontiers."""

        frontier_ids = [frontier.frontier_id for frontier in frontiers]

        if len(frontier_ids) != len(set(frontier_ids)):
            raise ValueError("Every frontier must have a unique ID.")

        if self.ROBOT_NODE_ID in frontier_ids:
            raise ValueError(f"Frontier ID {self.ROBOT_NODE_ID} is reserved for the robot.")

        positions = {
            self.ROBOT_NODE_ID: robot_position,
            **{frontier.frontier_id: (frontier.x, frontier.y) for frontier in frontiers},
        }

        graph = nx.Graph()

        for node_id, position in positions.items():
            graph.add_node(node_id, position=position)

        node_ids = list(positions.keys())

        # Connect every node pair so NetworkX can choose the lowest-cost MST edges.
        for index, node_a in enumerate(node_ids):
            for node_b in node_ids[index + 1:]:
                cost = self.travel_cost_function(positions[node_a], positions[node_b])

                # Ignore invalid costs instead of adding unusable graph edges.
                if not isfinite(cost) or cost < 0.0:
                    continue

                graph.add_edge(node_a, node_b, weight=cost)

        return graph

    @staticmethod
    def generate_mst(graph: nx.Graph) -> nx.Graph:
        """Use NetworkX Kruskal to generate the minimum spanning tree."""

        if graph.number_of_nodes() <= 1:
            return graph.copy()

        if not nx.is_connected(graph):
            raise ValueError("The graph is disconnected; not every frontier is reachable.")

        return nx.minimum_spanning_tree(graph, weight="weight", algorithm="kruskal")

    def recommended_traversal(self, mst: nx.Graph) -> List[int]:
        """Generate a depth-first order starting from the robot."""

        if self.ROBOT_NODE_ID not in mst:
            return []

        ordered_mst = nx.Graph()
        ordered_mst.add_nodes_from(mst.nodes(data=True))

        # Add shorter edges first so DFS prefers the cheaper connected branch.
        sorted_edges = sorted(mst.edges(data=True), key=lambda edge: edge[2]["weight"])
        ordered_mst.add_edges_from(sorted_edges)

        traversal_order = list(nx.dfs_preorder_nodes(ordered_mst, source=self.ROBOT_NODE_ID))
        return [node_id for node_id in traversal_order if node_id != self.ROBOT_NODE_ID]

    @staticmethod
    def order_frontiers(frontiers: List[Frontier], traversal_order: List[int]) -> List[Frontier]:
        """Arrange Frontier objects in the selected traversal order."""

        frontier_by_id = {frontier.frontier_id: frontier for frontier in frontiers}
        return [frontier_by_id[frontier_id] for frontier_id in traversal_order]

    @staticmethod
    def convert_mst_edges(mst: nx.Graph) -> List[Edge]:
        """Convert NetworkX edges into the planner's Edge objects."""

        return [
            Edge(node_a=node_a, node_b=node_b, cost=float(edge_data["weight"]))
            for node_a, node_b, edge_data in mst.edges(data=True)
        ]

    def plan(self, frontiers: List[Frontier], robot_position: Position) -> Tuple[List[Edge], List[Frontier]]:
        """Build the graph, generate its MST, and order the frontiers."""

        graph = self.construct_graph(frontiers, robot_position)
        mst = self.generate_mst(graph)
        traversal_order = self.recommended_traversal(mst)
        mst_edges = self.convert_mst_edges(mst)
        ordered_frontiers = self.order_frontiers(frontiers, traversal_order)
        return mst_edges, ordered_frontiers

    def generate_latest_plan(self, frontiers: List[Frontier], robot_position: Position) -> Tuple[List[Edge], List[Frontier]]:
        """Generate a plan from data already received by the Exploration Manager."""

        return self.plan(frontiers, robot_position)
