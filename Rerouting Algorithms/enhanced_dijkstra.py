"""
Enhanced AI Dijkstra Algorithm for Dynamic Vehicle Rerouting
Research Project: Dynamic Rerouting During Train Crossings

This module implements an enhanced Dijkstra's algorithm with AI features:
- Dynamic edge weight updates based on real-time conditions
- Predictive routing based on train schedules
- Traffic congestion awareness
- Multi-objective optimization (time, distance, congestion)
"""

import heapq
import math
from typing import Dict, List, Tuple, Optional, Set
from dataclasses import dataclass
from collections import defaultdict
import time


@dataclass
class Edge:
    """Represents a road edge in the network graph."""
    from_node: str
    to_node: str
    edge_id: str
    length: float
    max_speed: float
    base_weight: float  # Base travel time
    current_weight: float  # Current travel time (can be updated dynamically)
    is_blocked: bool = False  # True if blocked by train crossing
    congestion_factor: float = 1.0  # Multiplier for congestion (1.0 = no congestion)
    vehicle_count: int = 0  # Number of vehicles currently on this edge


@dataclass
class TrainCrossing:
    """Represents a train crossing event."""
    crossing_id: str
    edge_ids: Set[str]  # Edges blocked by the crossing
    start_time: float
    end_time: float
    severity: float = 1.0  # Severity multiplier (1.0 = normal, higher = more severe)


class EnhancedDijkstra:
    """
    Enhanced AI Dijkstra Algorithm with dynamic rerouting capabilities.
    
    Features:
    1. Dynamic weight updates based on real-time conditions
    2. Predictive routing using train schedules
    3. Traffic congestion awareness
    4. Multi-objective path optimization
    5. Real-time path recalculation
    """
    
    def __init__(self, network_graph: Dict[str, List[Edge]],
                 priority_mode: str = "balanced",
                 balanced_time_weight: float = 0.7,
                 balanced_distance_weight: float = 0.3):
        """
        Initialize the enhanced Dijkstra algorithm.
        
        Args:
            network_graph: Dictionary mapping node_id -> list of outgoing edges
        """
        self.graph = network_graph
        self.nodes = set()
        for node, edges in network_graph.items():
            self.nodes.add(node)
            for edge in edges:
                self.nodes.add(edge.to_node)
        
        # Dijkstra optimization preferences
        self.priority_mode = priority_mode
        self.balanced_time_weight, self.balanced_distance_weight = self._normalize_balanced_weights(
            balanced_time_weight, balanced_distance_weight
        )

        # AI Enhancement: Predictive train crossing data
        self.train_crossings: Dict[str, TrainCrossing] = {}
        
        # AI Enhancement: Historical traffic patterns
        self.historical_congestion: Dict[str, List[float]] = defaultdict(list)
        
        # AI Enhancement: Real-time traffic monitoring
        self.real_time_traffic: Dict[str, int] = defaultdict(int)
        
        # AI Enhancement: Edge importance scores (for prioritizing critical routes)
        self.edge_importance: Dict[str, float] = defaultdict(lambda: 1.0)
    
    def update_edge_weight(self, edge_id: str, new_weight: float, reason: str = ""):
        """
        Dynamically update the weight of an edge.
        
        Args:
            edge_id: The ID of the edge to update
            new_weight: New weight value
            reason: Reason for the update (for logging/debugging)
        """
        for edges in self.graph.values():
            for edge in edges:
                if edge.edge_id == edge_id:
                    edge.current_weight = new_weight
                    if reason:
                        print(f"Updated edge {edge_id} weight to {new_weight}: {reason}")
                    break
    
    def block_edge(self, edge_id: str, block: bool = True):
        """
        Block or unblock an edge (e.g., due to train crossing).
        
        Args:
            edge_id: The ID of the edge to block/unblock
            block: True to block, False to unblock
        """
        for edges in self.graph.values():
            for edge in edges:
                if edge.edge_id == edge_id:
                    edge.is_blocked = block
                    if block:
                        edge.current_weight = float('inf')
                    else:
                        edge.current_weight = edge.base_weight * edge.congestion_factor
                    break
    
    def register_train_crossing(self, crossing: TrainCrossing):
        """
        Register a train crossing event for predictive routing.
        
        Args:
            crossing: TrainCrossing object with crossing details
        """
        self.train_crossings[crossing.crossing_id] = crossing
        
        # Block all affected edges
        for edge_id in crossing.edge_ids:
            self.block_edge(edge_id, block=True)
    
    def remove_train_crossing(self, crossing_id: str):
        """
        Remove a train crossing event and unblock affected edges.
        
        Args:
            crossing_id: ID of the crossing to remove
        """
        if crossing_id in self.train_crossings:
            crossing = self.train_crossings[crossing_id]
            for edge_id in crossing.edge_ids:
                self.block_edge(edge_id, block=False)
            del self.train_crossings[crossing_id]
    
    def update_congestion(self, edge_id: str, vehicle_count: int, max_capacity: int = 15):
        """
        Update congestion factor for an edge based on vehicle count.
        Fix #3: max_capacity=15 (was 50) so congestion kicks in earlier for urban streets.

        Args:
            edge_id: The ID of the edge
            vehicle_count: Current number of vehicles on the edge
            max_capacity: Maximum capacity before severe congestion
        """
        for edges in self.graph.values():
            for edge in edges:
                if edge.edge_id == edge_id:
                    edge.vehicle_count = vehicle_count
                    # AI Enhancement: Non-linear congestion model
                    if vehicle_count >= max_capacity:
                        edge.congestion_factor = 3.0  # Severe congestion
                    elif vehicle_count >= max_capacity * 0.7:
                        edge.congestion_factor = 2.0  # Moderate congestion
                    elif vehicle_count >= max_capacity * 0.4:
                        edge.congestion_factor = 1.5  # Light congestion
                    else:
                        edge.congestion_factor = 1.0  # No congestion
                    
                    # Update current weight if not blocked
                    if not edge.is_blocked:
                        edge.current_weight = edge.base_weight * edge.congestion_factor
                    
                    # Store historical data for AI learning
                    self.historical_congestion[edge_id].append(edge.congestion_factor)
                    if len(self.historical_congestion[edge_id]) > 100:
                        self.historical_congestion[edge_id].pop(0)
                    break
    
    def predict_congestion(self, edge_id: str, future_time: float) -> float:
        """
        AI Enhancement: Predict congestion at a future time based on historical patterns.
        
        Args:
            edge_id: The ID of the edge
            future_time: Time in the future to predict for
            
        Returns:
            Predicted congestion factor
        """
        if edge_id not in self.historical_congestion or not self.historical_congestion[edge_id]:
            return 1.0
        
        # Simple moving average prediction (can be enhanced with ML models)
        recent_values = self.historical_congestion[edge_id][-20:]
        if recent_values:
            avg_congestion = sum(recent_values) / len(recent_values)
            # Add time-based variation (rush hour simulation)
            time_factor = 1.0 + 0.3 * math.sin((future_time / 3600) * 2 * math.pi)
            return avg_congestion * time_factor
        
        return 1.0
    
    def calculate_path(self, start_node: str, end_node: str,
                      current_time: float = 0.0,
                      avoid_edges: Optional[Set[str]] = None,
                      priority: Optional[str] = None) -> Tuple[List[str], float]:
        """
        Calculate optimal path using enhanced Dijkstra algorithm.
        
        Args:
            start_node: Starting node ID
            end_node: Destination node ID
            current_time: Current simulation time
            avoid_edges: Set of edge IDs to avoid
            priority: Optimization priority ("time", "distance", "balanced")
            
        Returns:
            Tuple of (path as list of edge IDs, total cost)
        """
        if start_node not in self.nodes or end_node not in self.nodes:
            return [], float('inf')
        
        if avoid_edges is None:
            avoid_edges = set()
        
        if not priority:
            priority = self.priority_mode

        # Priority queue: (cost, node, path)
        pq = [(0, start_node, [])]
        visited = set()
        distances = {start_node: 0}
        previous = {}
        
        while pq:
            current_cost, current_node, current_path = heapq.heappop(pq)
            
            if current_node in visited:
                continue
            
            visited.add(current_node)
            
            if current_node == end_node:
                # Reconstruct path
                path_edges = []
                node = end_node
                while node in previous:
                    edge = previous[node]
                    path_edges.insert(0, edge.edge_id)
                    node = edge.from_node
                return path_edges, current_cost
            
            # Explore neighbors
            if current_node in self.graph:
                for edge in self.graph[current_node]:
                    # Skip if edge should be avoided
                    if edge.edge_id in avoid_edges:
                        continue
                    
                    # Skip if edge is blocked
                    if edge.is_blocked:
                        continue
                    
                    neighbor = edge.to_node
                    
                    # Calculate edge cost based on priority
                    if priority == "time":
                        edge_cost = edge.current_weight
                    elif priority == "distance":
                        edge_cost = edge.length
                    else:  # balanced
                        # Multi-objective: combine time and distance
                        time_weight = self.balanced_time_weight
                        distance_weight = self.balanced_distance_weight
                        normalized_time = edge.current_weight / 100.0  # Normalize
                        normalized_distance = edge.length / 1000.0  # Normalize
                        edge_cost = (time_weight * normalized_time + 
                                   distance_weight * normalized_distance) * 100
                    
                    # AI Enhancement: Predictive cost adjustment
                    predicted_congestion = self.predict_congestion(
                        edge.edge_id, current_time + current_cost
                    )
                    edge_cost *= predicted_congestion
                    
                    # AI Enhancement: Edge importance factor
                    edge_cost /= self.edge_importance[edge.edge_id]
                    
                    new_cost = current_cost + edge_cost
                    
                    if neighbor not in distances or new_cost < distances[neighbor]:
                        distances[neighbor] = new_cost
                        previous[neighbor] = edge
                        heapq.heappush(pq, (new_cost, neighbor, current_path + [edge.edge_id]))
        
        return [], float('inf')  # No path found
    
    def find_alternative_path(self, start_node: str, end_node: str,
                             original_path: List[str],
                             blocked_edges: Set[str],
                             current_time: float = 0.0,
                             priority: Optional[str] = None) -> Tuple[List[str], float]:
        """
        Find an alternative path avoiding blocked edges.
        
        Args:
            start_node: Starting node ID
            end_node: Destination node ID
            original_path: The original path that is now blocked
            blocked_edges: Set of blocked edge IDs
            current_time: Current simulation time
            
        Returns:
            Tuple of (alternative path as list of edge IDs, total cost)
        """
        return self.calculate_path(
            start_node, end_node, current_time, 
            avoid_edges=blocked_edges, priority=priority
        )

    @staticmethod
    def _normalize_balanced_weights(time_weight: float, distance_weight: float) -> Tuple[float, float]:
        total = time_weight + distance_weight
        if total <= 0:
            return 0.7, 0.3
        return time_weight / total, distance_weight / total
    
    def get_affected_edges(self, crossing_id: str) -> Set[str]:
        """
        Get all edges affected by a train crossing.
        
        Args:
            crossing_id: ID of the train crossing
            
        Returns:
            Set of affected edge IDs
        """
        if crossing_id in self.train_crossings:
            return self.train_crossings[crossing_id].edge_ids
        return set()
    
    def update_edge_importance(self, edge_id: str, importance: float):
        """
        Update the importance score of an edge.
        Higher importance = lower cost (preferred routes).
        
        Args:
            edge_id: The ID of the edge
            importance: Importance score (1.0 = normal, >1.0 = more important)
        """
        self.edge_importance[edge_id] = max(0.1, importance)  # Minimum 0.1 to avoid division by zero


def create_graph_from_sumo_network(network_file: str) -> Dict[str, List[Edge]]:
    """
    Create a graph representation from SUMO network file.
    This is a simplified version - in practice, you'd parse the XML properly.
    
    Args:
        network_file: Path to SUMO .net.xml file
        
    Returns:
        Graph dictionary mapping node_id -> list of edges
    """
    # Note: This is a placeholder. In a real implementation, you would:
    # 1. Parse the SUMO network XML file
    # 2. Extract nodes and edges
    # 3. Calculate base weights from edge lengths and speeds
    # 4. Build the graph structure
    
    # For now, return empty graph - will be populated by TraCI
    return defaultdict(list)

