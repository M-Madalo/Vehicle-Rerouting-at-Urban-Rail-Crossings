"""
SUMO TraCI Controller for Dynamic Vehicle Rerouting
Handles real-time interaction with SUMO simulation, train detection, and vehicle rerouting.
"""

import traci
import traci.constants as tc
from typing import Dict, List, Set, Tuple, Optional
from collections import defaultdict
import xml.etree.ElementTree as ET
from enhanced_dijkstra import EnhancedDijkstra, Edge, TrainCrossing


class SUMOController:
    """
    Controller for SUMO simulation with train detection and dynamic rerouting.
    """
    
    def __init__(self, sumo_config: str, enhanced_dijkstra: EnhancedDijkstra, enable_rerouting: bool = True, rerouting_strategy: str = "intelligent", naive_hops: int = 3, train_crossing_duration: float = 180.0, train_id_whitelist: Optional[Set[str]] = None, approaching_lookahead_edges: int = 20, blocked_edge_effort: float = 999999.0, dijkstra_priority: str = "balanced", sumo_additional_args: Optional[List[str]] = None, debug_first_reroute: bool = False, debug_label: Optional[str] = None, block_only_grade_crossing_roads: bool = False):
        """
        Initialize SUMO controller.
        
        Args:
            sumo_config: Path to SUMO configuration file
            enhanced_dijkstra: Instance of EnhancedDijkstra algorithm
            enable_rerouting: If False, disables rerouting (baseline mode) but still collects metrics
            rerouting_strategy: "intelligent" (default) or "naive"
            naive_hops: Number of edge-to-edge hops for naive rerouting radius
            train_crossing_duration: How long a train crossing remains blocked (seconds)
            train_id_whitelist: Optional set of train IDs to include (others ignored)
            approaching_lookahead_edges: Number of edges to look ahead for approaching blocked areas
            blocked_edge_effort: Effort value applied to blocked edges
            dijkstra_priority: Dijkstra optimization priority ("time", "distance", "balanced")
            sumo_additional_args: Additional SUMO command arguments (e.g., ["--scale", "1.5"])
            debug_first_reroute: If True, logs the first reroute path and cost
            debug_label: Optional label printed with debug logs
        """
        self.sumo_config = sumo_config
        self.dijkstra = enhanced_dijkstra
        self.sumo_binary = "sumo-gui"  # or "sumo-gui" for visualization
        self.enable_rerouting = enable_rerouting  # Flag to enable/disable rerouting
        self.rerouting_strategy = rerouting_strategy
        self.naive_hops = max(1, int(naive_hops))
        self.train_crossing_duration = float(train_crossing_duration)
        self.train_id_whitelist = set(train_id_whitelist) if train_id_whitelist else None
        self.approaching_lookahead_edges = max(1, int(approaching_lookahead_edges))
        self.blocked_edge_effort = float(blocked_edge_effort)
        self.dijkstra_priority = dijkstra_priority
        self.sumo_additional_args = sumo_additional_args or []
        self.debug_first_reroute = debug_first_reroute
        self.debug_label = debug_label
        self.block_only_grade_crossing_roads = bool(block_only_grade_crossing_roads)
        
        # Track vehicles and their routes
        self.vehicle_routes: Dict[str, List[str]] = {}  # vehicle_id -> list of edge IDs
        self.vehicle_destinations: Dict[str, str] = {}  # vehicle_id -> destination edge
        self.vehicle_start_edges: Dict[str, str] = {}  # vehicle_id -> starting edge
        
        # Track train positions and crossings
        self.train_vehicles: Set[str] = set()
        self.active_crossings: Dict[str, TrainCrossing] = {}
        self.crossing_edges: Dict[str, Set[str]] = defaultdict(set)  # crossing_id -> edges
        
        # Track edge information from SUMO
        self.edge_info: Dict[str, Dict] = {}  # edge_id -> {length, speed, from_node, to_node}
        self.node_to_edges: Dict[str, List[str]] = defaultdict(list)  # node_id -> list of edge IDs
        
        # Statistics
        self.reroute_count = 0  # Total reroute operations (for debugging)
        self.unique_vehicles_rerouted = set()  # Track unique vehicles that have been rerouted
        self.total_vehicles_seen = set()  # Track all unique vehicles that have appeared
        self.total_vehicles = 0  # Current vehicle count
        self.vehicle_reroute_count: Dict[str, int] = {}  # Track how many times each vehicle was rerouted
        self.max_reroutes_per_vehicle = 3  # Maximum reroutes per vehicle to prevent excessive rerouting
        # Fix #6: Reroute cooldown to prevent route oscillation
        self.vehicle_last_reroute_time: Dict[str, float] = {}
        self.reroute_cooldown = 30.0  # Don't re-reroute the same vehicle within 30 seconds
        
        # Congestion and Travel Time Metrics
        self.vehicle_metrics: Dict[str, Dict] = {}  # vehicle_id -> metrics dict
        # Metrics tracked per vehicle:
        #   - start_time: when vehicle entered
        #   - original_route_length: original route distance
        #   - actual_distance: actual distance traveled
        #   - travel_time: total travel time
        #   - delay_time: time spent delayed (speed < threshold)
        #   - crossing_delay: time delayed specifically at crossings
        #   - was_rerouted: whether vehicle was rerouted
        #   - crossing_affected: whether vehicle was affected by crossing
        #   - max_speed: maximum speed achieved
        #   - avg_speed: average speed
        
        # Crossing-specific metrics
        self.crossing_metrics: Dict[str, Dict] = {}  # crossing_id -> metrics
        # Metrics per crossing:
        #   - vehicles_affected: number of vehicles affected
        #   - max_queue_length: maximum vehicles waiting
        #   - total_delay: total delay time for all vehicles
        #   - avg_delay_per_vehicle: average delay
        #   - duration: crossing duration
        
        # Edge congestion metrics (for edges near crossings)
        self.edge_congestion: Dict[str, List[float]] = defaultdict(list)  # edge_id -> list of vehicle counts over time
        self.crossing_edges_tracked: Set[str] = set()  # Edges to track for congestion
        
    def start_simulation(self):
        """Start the SUMO simulation."""
        sumo_cmd = [self.sumo_binary, "-c", self.sumo_config]
        if self.sumo_additional_args:
            sumo_cmd.extend(self.sumo_additional_args)
        
        if self.sumo_binary == "sumo-gui":
            print("Starting SUMO GUI (simulation will auto-run)...")
        else:
            print("Starting SUMO...")
        
        traci.start(sumo_cmd)
        print("SUMO simulation started")
        
        # For GUI mode, give it time to initialize and ensure it's ready
        if self.sumo_binary == "sumo-gui":
            print("   Waiting for GUI to initialize...")
            import time as time_module
            time_module.sleep(1.5)  # Give GUI time to open and render
            
            # Try to set GUI to play mode (unpause)
            try:
                # Set delay to 0 to make it run as fast as possible
                traci.gui.setDelay("View #0", 0)
                # Try to set the simulation to play (if supported)
                # Note: SUMO GUI doesn't have a direct "play" command via TraCI,
                # but setting delay to 0 helps it run automatically
            except:
                pass
            
            # Run a few initial steps to ensure GUI is active
            # This helps "wake up" the GUI and start the simulation
            for i in range(3):
                traci.simulationStep()
                time_module.sleep(0.1)  # Small delay between steps
            
            print("   [OK] GUI initialized")
            print("   [WARNING] If simulation is paused, click the Play button in SUMO GUI")
            print("   Note: The simulation will continue running in the background")
        
        # Initialize network information
        self._initialize_network()
        
        # Identify train vehicles
        self._identify_trains()
        
        # Check initial vehicle count
        initial_vehicles = len(traci.vehicle.getIDList())
        print(f"   Initial vehicles in simulation: {initial_vehicles}")
        if initial_vehicles == 0:
            print("   [WARNING] No vehicles at start - they will appear at their depart times")
    
    def _initialize_network(self):
        """Extract network information from SUMO."""
        print("Initializing network information...")
        
        # Get all edges
        all_edges = traci.edge.getIDList()
        
        # Build edge-to-edge connections by analyzing routes
        # In SUMO, edges connect when vehicles can travel from one to another
        # Store edge_connections as instance variable for dynamic building
        self.edge_connections = defaultdict(set)
        edge_connections = self.edge_connections  # Alias for compatibility
        
        edges_processed = 0
        edges_skipped = 0
        edges_failed = 0
        
        for edge_id in all_edges:
            # Only skip truly internal edges (start with ":") - these are junction-internal
            # All other edges we'll try to process
            if edge_id.startswith(':'):
                edges_skipped += 1
                continue
            
            try:
                # Get edge length - try multiple methods
                length = None
                
                # Method 1: Try edge.getLength() directly
                try:
                    length_result = traci.edge.getLength(edge_id)
                    # Check if it's a numeric type
                    if isinstance(length_result, (int, float)):
                        length = float(length_result)
                except:
                    pass
                
                # Method 2: If Method 1 failed or returned non-numeric, try getting from lanes
                if length is None:
                    try:
                        lanes = traci.edge.getLaneNumber(edge_id)
                        if lanes > 0:
                            lane_id = f"{edge_id}_0"
                            length = traci.lane.getLength(lane_id)
                    except:
                        pass
                
                # Method 3: If still no length, try to get from edge domain
                if length is None:
                    try:
                        # Some edges might have length in their domain
                        # Try accessing it differently
                        length_result = traci.edge.getLength(edge_id)
                        if hasattr(length_result, 'length'):
                            length = float(length_result.length)
                        elif isinstance(length_result, (list, tuple)) and len(length_result) > 0:
                            length = float(length_result[0])
                    except:
                        pass
                
                # If we still don't have a length, skip this edge
                if length is None or length <= 0:
                    edges_failed += 1
                    continue
                
                # Get max speed from lanes
                max_speed = 15.65  # Default 35 mph
                try:
                    lanes = traci.edge.getLaneNumber(edge_id)
                    if lanes > 0:
                        lane_id = f"{edge_id}_0"
                        try:
                            max_speed = traci.lane.getMaxSpeed(lane_id)
                        except:
                            # Try alternative method
                            try:
                                max_speed = traci.edge.getMaxSpeed(edge_id)
                            except:
                                max_speed = 15.65  # Default 35 mph
                    else:
                        max_speed = 15.65  # Default 35 mph
                except:
                    max_speed = 15.65  # Default 35 mph
                
                # Store edge information FIRST (before building connections)
                self.edge_info[edge_id] = {
                    'length': length,
                    'max_speed': max_speed,
                    'outgoing_edges': set()
                }
                edges_processed += 1
                
            except Exception as e:
                # Silently skip edges that can't be processed
                edges_skipped += 1
                continue
        
        if edges_skipped > 0:
            print(f"  Skipped {edges_skipped} internal edges (start with ':')")
        if edges_failed > 0:
            print(f"  Failed to process {edges_failed} edges (EdgeDomain or missing properties)")
        print(f"  Successfully processed {edges_processed} regular edges")
        
        # Build connections from vehicle routes
        routes_connections = 0
        try:
            all_vehicles = traci.vehicle.getIDList()
            # Process all vehicles to build complete graph
            for veh_id in all_vehicles:
                try:
                    route = traci.vehicle.getRoute(veh_id)
                    for i in range(len(route) - 1):
                        from_edge = route[i]
                        to_edge = route[i + 1]
                        # Only skip internal edges (starting with ":")
                        if from_edge.startswith(':') or to_edge.startswith(':'):
                            continue
                        if from_edge in self.edge_info and to_edge in self.edge_info:
                            edge_connections[from_edge].add(to_edge)
                            self.edge_info[from_edge]['outgoing_edges'].add(to_edge)
                            routes_connections += 1
                except:
                    pass
        except:
            pass
        
        # If no vehicles at start, we'll build connections from getOutgoing() only
        if routes_connections == 0:
            print(f"  No vehicles at initialization - building connections from network topology")
        
        # Build connections from SUMO's network topology using lane links (MORE RELIABLE)
        # Scan ALL lanes, not just lane 0, to catch turn connections from specific lanes.
        topology_connections = 0
        for edge_id in list(self.edge_info.keys()):
            try:
                num_lanes = traci.edge.getLaneNumber(edge_id)
                for lane_idx in range(num_lanes):
                    lane_id = f"{edge_id}_{lane_idx}"
                    try:
                        links = traci.lane.getLinks(lane_id)
                    except Exception:
                        continue
                    for link in links:
                        if link and len(link) > 0:
                            connected_lane = link[0]
                            connected_edge = connected_lane.rsplit('_', 1)[0]
                            if connected_edge in self.edge_info and not connected_edge.startswith(':'):
                                if connected_edge not in edge_connections.get(edge_id, set()):
                                    edge_connections[edge_id].add(connected_edge)
                                    self.edge_info[edge_id]['outgoing_edges'].add(connected_edge)
                                    topology_connections += 1
            except:
                pass
        
        print(f"  Built {topology_connections} connections from lane links (topology, all lanes)")
        
        print(f"  Built {routes_connections} connections from vehicle routes")
        
        # Build graph using edges as nodes (edge-to-edge graph)
        # This is more compatible with SUMO's structure
        # CRITICAL: Also populate dijkstra.nodes for pathfinding to work
        for from_edge, to_edges in edge_connections.items():
            if from_edge in self.edge_info:
                from_info = self.edge_info[from_edge]
                from_node = from_edge  # Use edge ID as node ID
                
                # Add from_node to dijkstra.nodes
                self.dijkstra.nodes.add(from_node)
                
                for to_edge in to_edges:
                    if to_edge in self.edge_info:
                        to_info = self.edge_info[to_edge]
                        to_node = to_edge
                        
                        # Add to_node to dijkstra.nodes
                        self.dijkstra.nodes.add(to_node)
                        
                        # Calculate base weight
                        base_weight = to_info['length'] / to_info['max_speed'] if to_info['max_speed'] > 0 else float('inf')
                        
                        # Create Edge object
                        edge = Edge(
                            from_node=from_node,
                            to_node=to_node,
                            edge_id=to_edge,  # The edge we're traveling on
                            length=to_info['length'],
                            max_speed=to_info['max_speed'],
                            base_weight=base_weight,
                            current_weight=base_weight
                        )
                        
                        # Add to graph
                        if from_node not in self.dijkstra.graph:
                            self.dijkstra.graph[from_node] = []
                        self.dijkstra.graph[from_node].append(edge)
        
        total_connections = sum(len(edges) for edges in self.dijkstra.graph.values())
        total_edge_connections = sum(len(edges) for edges in edge_connections.values())
        print(f"Initialized {len(self.edge_info)} edges with {total_connections} Dijkstra graph connections")
        print(f"  Total edge-to-edge connections found: {total_edge_connections}")
        
        if len(self.edge_info) == 0:
            print("[ERROR] No edges were successfully processed!")
            print("  This will cause rerouting to fail.")
        elif total_connections == 0:
            print("[WARNING] No connections in Dijkstra graph!")
            print(f"  Edge connections dictionary has {total_edge_connections} connections")
            print("  This means graph building failed - rerouting will not work!")
            print("  Possible causes:")
            print("    1. No vehicles at initialization to build routes from")
            print("    2. getOutgoing() not returning edges in edge_info")
            print("    3. All connections filtered out")
        
        if len(self.edge_info) == 0:
            print("[ERROR] No edges were successfully processed!")
            print("  This will cause rerouting to fail.")
            print("  Possible causes:")
            print("    1. Network file is corrupted or invalid")
            print("    2. All edges are special types (internal/junction)")
            print("    3. SUMO TraCI API issue")
    
    def _identify_trains(self):
        """Identify train vehicles in the simulation."""
        # Check existing vehicles
        all_vehicles = traci.vehicle.getIDList()
        
        for veh_id in all_vehicles:
            try:
                vtype = traci.vehicle.getTypeID(veh_id)
                if vtype == "train" or "train" in veh_id.lower():
                    self.train_vehicles.add(veh_id)
                    print(f"Identified train: {veh_id}")
            except:
                pass
    
    def update_train_list(self):
        """Continuously update the list of train vehicles (trains may enter later)."""
        all_vehicles = traci.vehicle.getIDList()
        
        for veh_id in all_vehicles:
            if veh_id not in self.train_vehicles:
                try:
                    vtype = traci.vehicle.getTypeID(veh_id)
                    if vtype == "train" or "train" in veh_id.lower():
                        self.train_vehicles.add(veh_id)
                        print(f"New train detected: {veh_id}")
                except:
                    pass
    
    def detect_train_crossings(self, current_time: float) -> List[TrainCrossing]:
        """
        Detect active train crossings at current time.
        
        Args:
            current_time: Current simulation time
            
        Returns:
            List of active TrainCrossing objects
        """
        active_crossings = []
        
        # Update train list to catch trains that entered later
        self.update_train_list()
        
        for train_id in list(self.train_vehicles):  # Use list copy to avoid modification during iteration
            try:
                if self.train_id_whitelist and train_id not in self.train_id_whitelist:
                    continue
                current_vehicles = traci.vehicle.getIDList()
                if train_id not in current_vehicles:
                    # Train has left, remove from tracking
                    self.train_vehicles.discard(train_id)
                    continue
                
                # Get train's current route and position
                route = traci.vehicle.getRoute(train_id)
                current_edge = traci.vehicle.getRoadID(train_id)
                
                if not current_edge:
                    continue
                
                # Get edges around the train (current and next few edges)
                train_edges = set()
                train_edges.add(current_edge)
                
                # Get upcoming edges in route
                if current_edge in route:
                    idx = route.index(current_edge)
                    # Include next 5 edges (train is long, ~2286m)
                    for i in range(idx, min(idx + 6, len(route))):
                        train_edges.add(route[i])
                
                # Find road edges that intersect with train route
                # Strategy: Find edges that share junctions with train edges
                affected_road_edges = set()
                
                # Known grade crossing road edges (identified from simulation)
                GRADE_CROSSING_ROADS = {
                    '591882550#1', '-591882550#1', 
                    '19444757#7', '-19444757#7', '19444757#8', '-19444757#8',
                    '19453446#2', '-19453446#2',
                    '48729817#2', '48729817#7',
                }
                
                # Method 1: Find road edges that share junctions with train edges
                for train_edge in train_edges:
                    try:
                        train_to_junction = traci.edge.getToJunction(train_edge)
                        train_from_junction = traci.edge.getFromJunction(train_edge)
                        train_junctions = {train_to_junction, train_from_junction}
                        
                        # Optionally limit blocking to grade crossing roads only
                        if not self.block_only_grade_crossing_roads:
                            # Find all road edges that connect to these junctions
                            for edge_id in self.edge_info.keys():
                                try:
                                    edge_to = traci.edge.getToJunction(edge_id)
                                    edge_from = traci.edge.getFromJunction(edge_id)
                                    # If this road edge shares a junction with train, block it
                                    if edge_to in train_junctions or edge_from in train_junctions:
                                        if edge_id not in train_edges:
                                            affected_road_edges.add(edge_id)
                                except:
                                    pass
                        
                        # Always check known grade crossing roads
                        for road in GRADE_CROSSING_ROADS:
                            if road in self.edge_info:
                                try:
                                    road_to = traci.edge.getToJunction(road)
                                    road_from = traci.edge.getFromJunction(road)
                                    if road_to in train_junctions or road_from in train_junctions:
                                        affected_road_edges.add(road)
                                except:
                                    pass
                    except:
                        pass
                
                # Use a stable crossing ID per train (not per second)
                crossing_id = f"train_crossing_{train_id}"
                
                # If crossing doesn't exist, create it
                if crossing_id not in self.active_crossings:
                    # Block both train edges AND affected road edges
                    all_blocked_edges = train_edges.copy()
                    all_blocked_edges.update(affected_road_edges)
                    
                    # Create new crossing
                    crossing = TrainCrossing(
                        crossing_id=crossing_id,
                        edge_ids=all_blocked_edges,
                        start_time=current_time,
                        end_time=current_time + self.train_crossing_duration,
                        severity=1.0
                    )
                    self.active_crossings[crossing_id] = crossing
                    self.dijkstra.register_train_crossing(crossing)
                    active_crossings.append(crossing)
                    print(f"\n[TRAIN] NEW TRAIN CROSSING DETECTED [TRAIN]")
                    print(f"   Train ID: {train_id}")
                    print(f"   Current Edge: {current_edge}")
                    print(f"   Train Edges: {len(train_edges)}")
                    print(f"   Affected Road Edges: {len(affected_road_edges)}")
                    print(f"   Total Blocked Edges: {len(all_blocked_edges)}")
                    if len(affected_road_edges) <= 10:
                        print(f"   Road Edges to Block: {list(affected_road_edges)}")
                    elif len(affected_road_edges) > 0:
                        print(f"   Sample Road Edges: {list(affected_road_edges)[:10]}")
                    print(f"   Crossing Duration: {self.train_crossing_duration:.0f} seconds")
                    print()
                else:
                    # Update existing crossing
                    crossing = self.active_crossings[crossing_id]
                    # Extend end time if train is still crossing
                    crossing.end_time = current_time + self.train_crossing_duration
                    # Update blocked edges
                    all_blocked_edges = train_edges | affected_road_edges
                    if all_blocked_edges != crossing.edge_ids:
                        # Update blocked edges
                        old_edges = crossing.edge_ids.copy()
                        crossing.edge_ids = all_blocked_edges
                        # Unblock old edges that are no longer blocked
                        for old_edge in old_edges - all_blocked_edges:
                            self.dijkstra.block_edge(old_edge, block=False)
                        # Block new edges
                        for new_edge in all_blocked_edges - old_edges:
                            self.dijkstra.block_edge(new_edge, block=True)
                    active_crossings.append(crossing)
                    
            except Exception as e:
                # Train might have issues, but continue
                if "train" in str(e).lower() or train_id:
                    pass  # Silently continue
                else:
                    print(f"Warning in train detection: {e}")
        
        # Remove expired crossings
        expired = []
        for crossing_id, crossing in self.active_crossings.items():
            if current_time > crossing.end_time:
                expired.append(crossing_id)
        
        for crossing_id in expired:
            crossing = self.active_crossings[crossing_id]
            # Unblock all edges
            for edge_id in crossing.edge_ids:
                self.dijkstra.block_edge(edge_id, block=False)
                # Reset effort values
                try:
                    traci.edge.setEffort(edge_id, -1.0)  # Reset to default
                except:
                    pass
            self.dijkstra.remove_train_crossing(crossing_id)
            del self.active_crossings[crossing_id]
            print(f"[OK] Train crossing ended: {crossing_id} (unblocked {len(crossing.edge_ids)} edges)")
        
        return active_crossings
    
    def get_vehicle_route_info(self, vehicle_id: str) -> Tuple[str, str, List[str]]:
        """
        Get vehicle's current route information.
        
        Args:
            vehicle_id: ID of the vehicle
            
        Returns:
            Tuple of (current_edge, destination_edge, route_edges)
            destination_edge is the ORIGINAL destination (not current route's last edge)
        """
        try:
            current_edge = traci.vehicle.getRoadID(vehicle_id)
            route = traci.vehicle.getRoute(vehicle_id)
            
            # Use stored original destination if available, otherwise use current route's last edge
            if vehicle_id in self.vehicle_destinations:
                destination_edge = self.vehicle_destinations[vehicle_id]
            elif route:
                destination_edge = route[-1]
            else:
                destination_edge = current_edge
            
            return current_edge, destination_edge, route
        except:
            return None, None, []
    
    def reroute_vehicle(self, vehicle_id: str, current_time: float, 
                       blocked_edges: Set[str]) -> bool:
        """
        Reroute a vehicle using enhanced Dijkstra algorithm.
        
        Args:
            vehicle_id: ID of the vehicle to reroute
            current_time: Current simulation time
            blocked_edges: Set of blocked edge IDs to avoid
            
        Returns:
            True if rerouting was successful, False otherwise
        """
        try:
            current_edge, destination_edge, original_route = self.get_vehicle_route_info(vehicle_id)
            
            if not current_edge or not destination_edge:
                return False

            # Helper to find first non-internal edge that exists in the graph
            def _first_non_internal(edge_iterable):
                for e in edge_iterable:
                    if isinstance(e, str) and not e.startswith(':') and e in self.edge_info:
                        return e
                return None
            
            # Estimate remaining route cost (rough time) to avoid bad detours
            def _estimate_remaining_cost(route_seq, start_edge, blocked_set):
                if not route_seq:
                    return 0.0
                try:
                    start_idx = route_seq.index(start_edge)
                except ValueError:
                    start_idx = 0
                wait_penalty = getattr(self, "reroute_blocked_wait_penalty", self.train_crossing_duration)
                total = 0.0
                for edge_id in route_seq[start_idx:]:
                    info = self.edge_info.get(edge_id)
                    if not info:
                        continue
                    base_time = info['length'] / info['max_speed'] if info['max_speed'] > 0 else 0.0
                    if edge_id in blocked_set:
                        base_time += float(wait_penalty)
                    total += base_time
                return total

            # If current edge is internal/not in graph, try to map to nearest valid edge from the route
            if current_edge not in self.edge_info and original_route:
                route_seq = list(original_route)
                if current_edge.startswith(':') and current_edge in route_seq:
                    try:
                        idx = route_seq.index(current_edge)
                        candidate = _first_non_internal(route_seq[idx + 1:])
                        if candidate:
                            current_edge = candidate
                    except Exception:
                        pass
                # If still not mapped, grab first non-internal anywhere in route
                if current_edge not in self.edge_info:
                    fallback = _first_non_internal(route_seq)
                    if fallback:
                        current_edge = fallback

            # If destination edge is internal/not in graph, map to the last valid edge in the route
            if destination_edge not in self.edge_info and original_route:
                route_seq = list(original_route)
                dest_fallback = _first_non_internal(reversed(route_seq))
                if dest_fallback:
                    destination_edge = dest_fallback

            # If still missing, use SUMO rerouteEffort as a safe fallback
            if current_edge not in self.edge_info or destination_edge not in self.edge_info:
                try:
                    traci.vehicle.rerouteEffort(vehicle_id)
                    self.reroute_count += 1
                    self.unique_vehicles_rerouted.add(vehicle_id)
                    self.vehicle_reroute_count[vehicle_id] = self.vehicle_reroute_count.get(vehicle_id, 0) + 1
                    return True
                except:
                    if self.reroute_count < 10:
                        missing = current_edge if current_edge not in self.edge_info else destination_edge
                        print(f"[WARNING] Reroute failed for {vehicle_id}: Edge '{missing}' not in graph (after fallback)")
                    return False
            
            # In our edge-to-edge graph, nodes are edge IDs
            from_node = current_edge  # Current edge is the starting node
            to_node = destination_edge  # Destination edge is the target node
            
            # Calculate new path
            new_path, cost = self.dijkstra.find_alternative_path(
                from_node, to_node, original_route, blocked_edges, current_time,
                priority=self.dijkstra_priority
            )

            # Skip reroute if alternative is significantly worse than staying
            if new_path and cost < float('inf'):
                route_seq = list(original_route) if original_route else []
                remaining_cost = _estimate_remaining_cost(route_seq, current_edge, blocked_edges)
                ratio_threshold = getattr(self, "reroute_cost_ratio_threshold", 1.25)
                if remaining_cost > 0 and cost > remaining_cost * ratio_threshold:
                    self._last_reroute_skip_reason = "cost"
                    return False

            # region agent log
            try:
                import json as _agent_json, time as _agent_time
                _agent_payload = {
                    "id": f"log_{int(_agent_time.time() * 1000)}",
                    "timestamp": int(_agent_time.time() * 1000),
                    "location": "sumo_controller.py:reroute_vehicle",
                    "message": "pathfinding_result",
                    "runId": "run1",
                    "hypothesisId": ["H3", "H4"],
                    "data": {
                        "vehicle_id": vehicle_id,
                        "from_node": from_node,
                        "to_node": to_node,
                        "blocked_edges": len(blocked_edges),
                        "new_path_len": len(new_path) if new_path else 0,
                        "cost": cost,
                        "graph_nodes": len(self.dijkstra.nodes),
                        "graph_connections": sum(len(edges) for edges in self.dijkstra.graph.values()),
                        "current_edge_in_graph": current_edge in self.edge_info,
                        "dest_edge_in_graph": destination_edge in self.edge_info
                    }
                }
                with open(r"c:\Users\25562\OneDrive - Tennessee State University\MAXPRESSURE Project\FRA\.cursor\debug.log", "a", encoding="utf-8") as _agent_log:
                    _agent_log.write(_agent_json.dumps(_agent_payload) + "\n")
            except Exception:
                pass
            # endregion

            # Debug pathfinding results
            if self.reroute_count < 10:
                if not new_path:
                    print(f"[WARNING] Pathfinding returned no path for {vehicle_id}")
                    print(f"   From: {from_node}")
                    print(f"   To: {to_node}")
                    print(f"   Blocked edges: {len(blocked_edges)}")
                    print(f"   This usually means:")
                    print(f"   1. No path exists from current to destination")
                    print(f"   2. Graph connectivity issue - edges not properly connected")
                    print(f"   3. All paths blocked by train crossings")
                elif cost >= float('inf'):
                    print(f"[WARNING] Pathfinding returned infinite cost for {vehicle_id}")
                    print(f"   This means no valid path exists (all paths blocked)")
                elif len(new_path) == 0:
                    print(f"[WARNING] Pathfinding returned empty path for {vehicle_id}")
                    print(f"   This is unusual - check Dijkstra algorithm")
            
            if new_path and cost < float('inf') and len(new_path) > 0:
                if self.debug_first_reroute and self.reroute_count == 0:
                    label = f"{self.debug_label} " if self.debug_label else ""
                    path_preview = new_path[:10]
                    print(
                        f"[DEBUG] {label}first reroute: priority={self.dijkstra_priority}, "
                        f"cost={cost:.2f}, path_len={len(new_path)}, "
                        f"preview={path_preview}"
                    )
                # Ensure the route starts from current edge
                if new_path[0] != current_edge:
                    new_path.insert(0, current_edge)
                
                # Visualize the rerouting event (only for first few reroutes to avoid too many files)
                try:
                    from visualize_dijkstra_graph import visualize_rerouting_event
                    visualize_rerouting_event(
                        controller=self,
                        vehicle_id=vehicle_id,
                        current_edge=current_edge,
                        destination_edge=destination_edge,
                        blocked_edges=blocked_edges,
                        new_path=new_path,
                        current_time=current_time,
                        max_visualizations=5,  # Only visualize first 5 rerouting events
                        show_possible_routes=True  # Enable showing alternative routes
                    )
                except Exception as e:
                    # Don't fail rerouting if visualization fails
                    if self.reroute_count < 3:
                        print(f"[WARNING] Visualization failed: {e}")
                
                # Update route in SUMO
                try:
                    # First try to set route directly
                    try:
                        # Filter out any invalid edges
                        valid_path = [e for e in new_path if e in self.edge_info or e == current_edge]
                        if len(valid_path) > 1:  # Need at least 2 edges for a route
                            traci.vehicle.setRoute(vehicle_id, valid_path)
                            self.vehicle_routes[vehicle_id] = valid_path
                            self.reroute_count += 1
                            self.unique_vehicles_rerouted.add(vehicle_id)
                            if vehicle_id not in self.vehicle_reroute_count:
                                self.vehicle_reroute_count[vehicle_id] = 0
                            self.vehicle_reroute_count[vehicle_id] += 1
                            if self.reroute_count <= 5 or self.reroute_count % 20 == 0:
                                print(f"[OK] Rerouted vehicle {vehicle_id} (new route: {len(valid_path)} edges, cost: {cost:.2f})")
                            return True
                    except Exception as e:
                        # If setRoute fails, try rerouteEffort
                        pass
                    
                    # Fallback: Use SUMO's built-in rerouting
                    traci.vehicle.rerouteEffort(vehicle_id)
                    self.reroute_count += 1
                    self.unique_vehicles_rerouted.add(vehicle_id)
                    if vehicle_id not in self.vehicle_reroute_count:
                        self.vehicle_reroute_count[vehicle_id] = 0
                    self.vehicle_reroute_count[vehicle_id] += 1
                    if self.reroute_count <= 5 or self.reroute_count % 20 == 0:
                        print(f"[OK] Rerouted vehicle {vehicle_id} (using SUMO rerouteEffort)")
                    return True
                except Exception as e:
                    # Silently fail to avoid spam
                    return False
            else:
                # Try SUMO's built-in rerouting as fallback
                try:
                    traci.vehicle.rerouteEffort(vehicle_id)
                    self.reroute_count += 1
                    self.unique_vehicles_rerouted.add(vehicle_id)
                    if vehicle_id not in self.vehicle_reroute_count:
                        self.vehicle_reroute_count[vehicle_id] = 0
                    self.vehicle_reroute_count[vehicle_id] += 1
                    if self.reroute_count <= 5:
                        print(f"[OK] Rerouted vehicle {vehicle_id} (fallback method)")
                    return True
                except:
                    return False
                
        except Exception as e:
            # Silently handle errors to avoid spam
            return False

    def _build_near_crossing_edges(self, active_crossings: List[TrainCrossing]) -> Set[str]:
        """Build a set of edges near active crossings (used for naive rerouting)."""
        near_edges: Set[str] = set()
        for crossing in active_crossings:
            near_edges.update(crossing.edge_ids)
        # Expand multiple hops to widen the naive trigger radius
        max_hops = self.naive_hops
        frontier = set(near_edges)
        for _ in range(max_hops):
            next_frontier = set()
            for edge_id in frontier:
                try:
                    incoming = traci.edge.getIncoming(edge_id)
                    next_frontier.update([e for e in incoming if isinstance(e, str) and not e.startswith(':')])
                except:
                    pass
                try:
                    outgoing = traci.edge.getOutgoing(edge_id)
                    next_frontier.update([e for e in outgoing if isinstance(e, str) and not e.startswith(':')])
                except:
                    pass
            # Only keep newly discovered edges in the next frontier
            next_frontier -= near_edges
            near_edges.update(next_frontier)
            frontier = next_frontier
            if not frontier:
                break
        return near_edges

    def _is_vehicle_near_crossing(self, current_edge: str, route: List[str], near_edges: Set[str]) -> bool:
        """Return True if a vehicle is near any active crossing (naive routing rule)."""
        if not current_edge:
            return False
        if current_edge in near_edges:
            return True
        if route and current_edge in route:
            try:
                idx = route.index(current_edge)
                lookahead = route[idx:min(idx + 6, len(route))]
                for edge_id in lookahead:
                    if edge_id in near_edges:
                        return True
            except:
                pass
        return False
    
    def update_traffic_congestion(self):
        """Update congestion information for all edges."""
        all_edges = traci.edge.getIDList()
        
        for edge_id in all_edges:
            try:
                vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
                self.dijkstra.update_congestion(edge_id, vehicle_count)
                
                # Track congestion for edges near crossings
                if edge_id in self.crossing_edges_tracked:
                    self.edge_congestion[edge_id].append(vehicle_count)
            except:
                pass
    
    def _update_vehicle_metrics(self, current_time: float, active_crossings: List, blocked_edges: Set[str]):
        """Update metrics for all vehicles in the simulation."""
        all_vehicles = traci.vehicle.getIDList()
        
        for vehicle_id in all_vehicles:
            if vehicle_id in self.train_vehicles:
                continue
                
            try:
                # Initialize metrics if vehicle is new
                if vehicle_id not in self.vehicle_metrics:
                    route = traci.vehicle.getRoute(vehicle_id)
                    original_length = sum(self.edge_info.get(e, {}).get('length', 0) for e in route if e in self.edge_info)
                    
                    self.vehicle_metrics[vehicle_id] = {
                        'start_time': current_time,
                        'original_route_length': original_length,
                        'actual_distance': 0.0,
                        'travel_time': 0.0,
                        'delay_time': 0.0,  # Time spent at low speed (< 5 m/s)
                        'crossing_delay': 0.0,  # Time delayed specifically at crossings
                        'was_rerouted': vehicle_id in self.unique_vehicles_rerouted,
                        'crossing_affected': False,
                        'max_speed': 0.0,
                        'speed_sum': 0.0,
                        'speed_samples': 0,
                        'last_edge': None,
                        'last_position': None,
                        'in_crossing_area': False,
                        'crossing_start_time': None
                    }
                
                metrics = self.vehicle_metrics[vehicle_id]
                
                # Check if vehicle still exists
                try:
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                    position = traci.vehicle.getLanePosition(vehicle_id)
                    distance = traci.vehicle.getDistance(vehicle_id)
                    
                    # Update speed metrics
                    metrics['max_speed'] = max(metrics['max_speed'], speed)
                    metrics['speed_sum'] += speed
                    metrics['speed_samples'] += 1
                    
                    # Update distance (use SUMO's distance measurement)
                    metrics['actual_distance'] = distance
                    
                    # Update travel time
                    metrics['travel_time'] = current_time - metrics['start_time']
                    
                    # Check if vehicle is delayed (speed < 5 m/s = 18 km/h, typical congestion threshold)
                    if speed < 5.0:
                        metrics['delay_time'] += 1.0  # Add 1 second (assuming 1s step)
                    
                    # Check if vehicle is in crossing area
                    is_in_crossing = current_edge in blocked_edges or any(
                        current_edge in crossing.edge_ids for crossing in active_crossings
                    )
                    
                    if is_in_crossing and not metrics['in_crossing_area']:
                        # Entered crossing area
                        metrics['in_crossing_area'] = True
                        metrics['crossing_start_time'] = current_time
                        metrics['crossing_affected'] = True
                    elif is_in_crossing and metrics['in_crossing_area']:
                        # Still in crossing area - accumulate delay
                        if speed < 5.0:
                            metrics['crossing_delay'] += 1.0
                    elif not is_in_crossing and metrics['in_crossing_area']:
                        # Left crossing area
                        metrics['in_crossing_area'] = False
                        metrics['crossing_start_time'] = None
                    
                    # Update was_rerouted flag
                    metrics['was_rerouted'] = vehicle_id in self.unique_vehicles_rerouted
                    
                    metrics['last_edge'] = current_edge
                    metrics['last_position'] = position
                    
                except:
                    # Vehicle might have left simulation
                    pass
                    
            except:
                pass
    
    def _update_crossing_metrics(self, current_time: float, active_crossings: List, vehicles_to_check: List[str]):
        """Update metrics specific to train crossings."""
        for crossing in active_crossings:
            crossing_id = crossing.crossing_id
            
            # Initialize crossing metrics if new
            if crossing_id not in self.crossing_metrics:
                self.crossing_metrics[crossing_id] = {
                    'vehicles_affected': set(),
                    'max_queue_length': 0,
                    'total_delay': 0.0,
                    'start_time': current_time,
                    'duration': 0.0,
                    'blocked_edges': crossing.edge_ids.copy()
                }
                # Track these edges for congestion monitoring
                self.crossing_edges_tracked.update(crossing.edge_ids)
            
            metrics = self.crossing_metrics[crossing_id]
            
            # Count vehicles currently in crossing area AND vehicles whose routes intersect
            vehicles_in_area = 0
            for vehicle_id in vehicles_to_check:
                try:
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                    route = traci.vehicle.getRoute(vehicle_id)
                    
                    # Check if vehicle is currently on crossing edge
                    if current_edge in crossing.edge_ids:
                        vehicles_in_area += 1
                        metrics['vehicles_affected'].add(vehicle_id)
                        
                        # Check if vehicle is delayed (low speed)
                        speed = traci.vehicle.getSpeed(vehicle_id)
                        if speed < 5.0:
                            metrics['total_delay'] += 1.0  # 1 second delay
                    
                    # Also check if vehicle's route (current or original) intersects crossing
                    if route:
                        route_edges = set(route)
                        if route_edges & crossing.edge_ids:
                            metrics['vehicles_affected'].add(vehicle_id)
                    
                    # Check original route if available
                    if vehicle_id in self.vehicle_routes:
                        original_route_edges = set(self.vehicle_routes[vehicle_id])
                        if original_route_edges & crossing.edge_ids:
                            metrics['vehicles_affected'].add(vehicle_id)
                    
                    # Also check if vehicle would pass through crossing area based on route
                    # Check if any edge in vehicle's route is near/connected to crossing edges
                    if route:
                        for route_edge in route[:min(50, len(route))]:  # Check first 50 edges
                            # Check if this edge connects to any crossing edge
                            try:
                                outgoing = traci.edge.getOutgoing(route_edge)
                                incoming = traci.edge.getIncoming(route_edge)
                                all_connections = set(outgoing) | set(incoming)
                                if all_connections & crossing.edge_ids:
                                    metrics['vehicles_affected'].add(vehicle_id)
                                    break
                            except:
                                pass
                            
                except:
                    pass
            
            # Update max queue length
            metrics['max_queue_length'] = max(metrics['max_queue_length'], vehicles_in_area)
            metrics['duration'] = current_time - metrics['start_time']
    
    def _build_connections_from_vehicles(self):
        """Build graph connections dynamically from vehicle routes."""
        connections_added = 0
        vehicles_processed = 0
        routes_without_edges = 0
        
        try:
            all_vehicles = traci.vehicle.getIDList()
            for veh_id in all_vehicles:
                try:
                    route = traci.vehicle.getRoute(veh_id)
                    if not route:
                        continue
                    
                    vehicles_processed += 1
                    route_edges_in_graph = 0
                    
                    for i in range(len(route) - 1):
                        from_edge = route[i]
                        to_edge = route[i + 1]
                        
                        # Skip internal edges
                        if from_edge.startswith(':') or to_edge.startswith(':'):
                            continue
                        
                        # Only add if both edges are in our graph
                        if from_edge in self.edge_info and to_edge in self.edge_info:
                            route_edges_in_graph += 1
                            # Check if connection already exists
                            if from_edge not in self.dijkstra.graph or \
                               not any(e.edge_id == to_edge for e in self.dijkstra.graph.get(from_edge, [])):
                                
                                # Add to edge_connections (for tracking)
                                if from_edge not in self.edge_connections:
                                    self.edge_connections[from_edge] = set()
                                self.edge_connections[from_edge].add(to_edge)
                                
                                # Add to edge_info outgoing edges
                                self.edge_info[from_edge]['outgoing_edges'].add(to_edge)
                                
                                # Build Edge object and add to Dijkstra graph
                                from_info = self.edge_info[from_edge]
                                to_info = self.edge_info[to_edge]
                                
                                base_weight = to_info['length'] / to_info['max_speed'] if to_info['max_speed'] > 0 else float('inf')
                                
                                edge = Edge(
                                    from_node=from_edge,
                                    to_node=to_edge,
                                    edge_id=to_edge,
                                    length=to_info['length'],
                                    max_speed=to_info['max_speed'],
                                    base_weight=base_weight,
                                    current_weight=base_weight
                                )
                                
                                if from_edge not in self.dijkstra.graph:
                                    self.dijkstra.graph[from_edge] = []
                                self.dijkstra.graph[from_edge].append(edge)
                                
                                # Update Dijkstra nodes set
                                self.dijkstra.nodes.add(from_edge)
                                self.dijkstra.nodes.add(to_edge)
                                
                                connections_added += 1
                except:
                    pass
        except:
            pass
        
        return connections_added
    
    def run_step(self, current_time: float):
        """
        Run one simulation step.
        
        Args:
            current_time: Current simulation time
        """
        # Advance simulation (this is critical - vehicles won't appear without stepping)
        traci.simulationStep()
        
        # Debug: Check vehicle count at startup
        if current_time == 0.0:
            vehicle_count = len(traci.vehicle.getIDList())
            print(f"   Initial vehicle count: {vehicle_count}")
            if vehicle_count == 0:
                print("   [WARNING] No vehicles found at start - they will appear at their depart times")
        
        # Build connections dynamically from vehicle routes (if graph is empty or sparse)
        # Build aggressively before first train (t=300), then periodically every 60 s
        total_connections = sum(len(edges) for edges in self.dijkstra.graph.values())
        should_build = False
        if current_time < 300.0 and (total_connections < 100 or int(current_time) % 30 == 0):
            should_build = True
        elif int(current_time) % 60 == 0:
            should_build = True
        elif len(self.dijkstra.graph) == 0 or total_connections < 10:
            should_build = True

        if should_build:
            connections_added = self._build_connections_from_vehicles()
            if connections_added > 0 and current_time < 60.0:
                total_connections = sum(len(edges) for edges in self.dijkstra.graph.values())
                print(f"[INFO] Built {connections_added} new connections (total: {total_connections})")

        # Fix #3: Update congestion BEFORE crossing detection so rerouting uses current weights
        self.update_traffic_congestion()

        # Detect train crossings (this also updates train list)
        active_crossings = self.detect_train_crossings(current_time)

        # region agent log
        try:
            import json as _agent_json, time as _agent_time
            _agent_train_edges = {e for e in active_crossings for e in e.edge_ids if any(t in str(e) for t in self.train_vehicles) or "rail" in str(e).lower()} if active_crossings else set()
            _agent_payload = {
                "id": f"log_{int(_agent_time.time() * 1000)}",
                "timestamp": int(_agent_time.time() * 1000),
                "location": "sumo_controller.py:run_step",
                "message": "routing_snapshot",
                "runId": "run1",
                "hypothesisId": ["H1", "H2"],
                "data": {
                    "current_time": current_time,
                    "active_crossings": len(active_crossings),
                    "blocked_edges": len(blocked_edges),
                    "train_edges": len(_agent_train_edges),
                    "graph_nodes": len(self.dijkstra.nodes),
                    "graph_connections": sum(len(edges) for edges in self.dijkstra.graph.values())
                }
            }
            with open(r"c:\Users\25562\OneDrive - Tennessee State University\MAXPRESSURE Project\FRA\.cursor\debug.log", "a", encoding="utf-8") as _agent_log:
                _agent_log.write(_agent_json.dumps(_agent_payload) + "\n")
        except Exception:
            pass
        # endregion

        # Collect all blocked edges
        blocked_edges = set()
        for crossing in active_crossings:
            blocked_edges.update(crossing.edge_ids)

        # Get all vehicles (excluding trains)
        all_vehicles = traci.vehicle.getIDList()
        vehicles_to_check = [v for v in all_vehicles if v not in self.train_vehicles]
        
        # Track metrics for all vehicles
        self._update_vehicle_metrics(current_time, active_crossings, blocked_edges)
        
        # Track crossing-specific metrics
        self._update_crossing_metrics(current_time, active_crossings, vehicles_to_check)
        
        # Track all unique vehicles we've seen and store their original destinations
        for veh_id in vehicles_to_check:
            self.total_vehicles_seen.add(veh_id)
            # Store original destination and start edge when vehicle first appears
            if veh_id not in self.vehicle_destinations:
                try:
                    route = traci.vehicle.getRoute(veh_id)
                    if route:
                        self.vehicle_destinations[veh_id] = route[-1]  # Store original destination
                        self.vehicle_start_edges[veh_id] = route[0]  # Store original start edge
                        self.vehicle_routes[veh_id] = route.copy()  # Store original route
                except:
                    pass
        
        # Track vehicles that have been rerouted to avoid duplicate rerouting
        rerouted_this_step = set()

        # For naive rerouting, build a near-crossing edge set once per step
        near_crossing_edges = set()
        if active_crossings and self.rerouting_strategy == "naive":
            near_crossing_edges = self._build_near_crossing_edges(active_crossings)
        
        # Set high effort on blocked edges to discourage their use (only when crossings are active)
        # NOTE: In baseline mode (enable_rerouting=False), we still set effort but don't reroute
        if active_crossings and len(blocked_edges) > 0:
            try:
                for edge_id in blocked_edges:
                    try:
                        # Set high travel time (effort) on blocked edges
                        # This makes SUMO's rerouteEffort avoid these edges
                        traci.edge.setEffort(edge_id, self.blocked_edge_effort)  # Very high effort
                    except:
                        pass
                
                # REMOVED: Aggressive proactive rerouting that rerouted ALL vehicles
                # This was causing excessive rerouting and worse performance
                # Instead, we only reroute vehicles that actually need it (checked below)
            except:
                pass
        
        # Check each vehicle if it needs rerouting
        vehicles_need_reroute = 0
        reroute_failures = 0
        reroute_skipped = 0
        for vehicle_id in vehicles_to_check:
            try:
                current_edge, destination_edge, route = self.get_vehicle_route_info(vehicle_id)
                
                if not current_edge or not route:
                    continue
                
                # Check if vehicle's current edge or future route intersects with blocked edges
                route_edges = set(route)
                
                # Check if current position or future route intersects with blocked edges
                route_intersects = bool(route_edges & blocked_edges)
                
                # Also check original route (for metrics tracking)
                original_route_intersects = False
                if vehicle_id in self.vehicle_routes:
                    original_route_edges = set(self.vehicle_routes[vehicle_id])
                    original_route_intersects = bool(original_route_edges & blocked_edges)
                    # Mark as crossing affected if original route would intersect
                    if original_route_intersects and vehicle_id in self.vehicle_metrics:
                        self.vehicle_metrics[vehicle_id]['crossing_affected'] = True
                
                # Also check if vehicle is currently on a blocked edge
                currently_blocked = current_edge in blocked_edges
                
                # IMPORTANT: Check if vehicle is stopped/slowed near crossing (at gate/stop)
                # This detects vehicles waiting at crossing gates
                stopped_at_crossing = False
                try:
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    if speed < 2.0:  # Stopped or nearly stopped
                        # Check if vehicle is at same junction as train edges
                        if active_crossings:
                            try:
                                veh_to_junction = traci.edge.getToJunction(current_edge)
                                veh_from_junction = traci.edge.getFromJunction(current_edge)
                                
                                for crossing in active_crossings:
                                    for train_edge in crossing.edge_ids:
                                        try:
                                            train_to = traci.edge.getToJunction(train_edge)
                                            train_from = traci.edge.getFromJunction(train_edge)
                                            
                                            # If vehicle is at same junction as train, it's stopped at crossing
                                            if (veh_to_junction == train_to or veh_from_junction == train_to or
                                                veh_to_junction == train_from or veh_from_junction == train_from):
                                                stopped_at_crossing = True
                                                break
                                        except:
                                            pass
                                    if stopped_at_crossing:
                                        break
                            except:
                                pass
                except:
                    pass
                
                # Check if vehicle is approaching blocked area (enhanced lookahead)
                approaching_blocked = False
                if current_edge in route:
                    try:
                        idx = route.index(current_edge)
                        # ENHANCED: Check next edges in route to reroute early
                        upcoming_edges = set(route[idx:min(idx + self.approaching_lookahead_edges, len(route))])
                        approaching_blocked = bool(upcoming_edges & blocked_edges)
                    except:
                        pass
                
                # Determine if vehicle needs rerouting
                # Intelligent mode: reroute only when the route intersects or is approaching blockage
                # Naive mode: reroute any vehicle near the crossing area, regardless of route
                if self.rerouting_strategy == "naive" and active_crossings:
                    needs_reroute = self._is_vehicle_near_crossing(current_edge, route, near_crossing_edges)
                    if needs_reroute and vehicle_id in self.vehicle_metrics:
                        self.vehicle_metrics[vehicle_id]['crossing_affected'] = True
                else:
                    # Vehicle needs rerouting if:
                    # 1. Route intersects blocked edges, OR
                    # 2. Currently on blocked edge, OR
                    # 3. Approaching blocked area, OR
                    # 4. Stopped at crossing gate (waiting for train)
                    needs_reroute = (route_intersects or currently_blocked or 
                                   approaching_blocked or stopped_at_crossing)
                
                # FALLBACK: If there are active crossings but no affected road edges detected,
                # check if vehicle is near train edges by checking if any train edge is in vehicle's route
                # or if vehicle's route passes through junctions near train edges
                near_train_crossing = False
                if active_crossings and len(blocked_edges) > 0 and not needs_reroute:
                    # Get all train edges (edges that contain "rail" or are in train routes)
                    train_edges_only = {e for e in blocked_edges if any(train_id in str(e) for train_id in self.train_vehicles) or "rail" in str(e).lower()}
                    
                    # Check if vehicle route is near any train edge
                    # Method: Check if vehicle's route shares any junctions with train edges
                    try:
                        # Get junctions from vehicle's route edges
                        for route_edge in route[:min(20, len(route))]:  # Check first 20 edges of route
                            if route_edge in self.edge_info:
                                # Check if this edge connects to any train edge
                                try:
                                    outgoing = traci.edge.getOutgoing(route_edge)
                                    incoming = traci.edge.getIncoming(route_edge)
                                    all_connections = set(outgoing) | set(incoming)
                                    if all_connections & train_edges_only:
                                        near_train_crossing = True
                                        break
                                except:
                                    pass
                    except:
                        pass
                
                # Only reroute if vehicle actually needs it AND hasn't been rerouted this step
                # SKIP REROUTING IN BASELINE MODE
                if (needs_reroute or near_train_crossing) and vehicle_id not in rerouted_this_step:
                    vehicles_need_reroute += 1

                    # region agent log
                    try:
                        import json as _agent_json, time as _agent_time
                        _agent_payload = {
                            "id": f"log_{int(_agent_time.time() * 1000)}",
                            "timestamp": int(_agent_time.time() * 1000),
                            "location": "sumo_controller.py:run_step:decision",
                            "message": "reroute_decision",
                            "runId": "run1",
                            "hypothesisId": ["H2", "H3"],
                            "data": {
                                "vehicle_id": vehicle_id,
                                "current_time": current_time,
                                "needs_reroute": needs_reroute,
                                "near_train_crossing": near_train_crossing,
                                "route_intersects": route_intersects,
                                "currently_blocked": currently_blocked,
                                "approaching_blocked": approaching_blocked,
                                "stopped_at_crossing": stopped_at_crossing,
                                "blocked_edges": len(blocked_edges)
                            }
                        }
                        with open(r"c:\Users\25562\OneDrive - Tennessee State University\MAXPRESSURE Project\FRA\.cursor\debug.log", "a", encoding="utf-8") as _agent_log:
                            _agent_log.write(_agent_json.dumps(_agent_payload) + "\n")
                    except Exception:
                        pass
                    # endregion

                    # Fix #6: Skip if within reroute cooldown (prevents oscillation)
                    last_reroute = self.vehicle_last_reroute_time.get(vehicle_id, -999.0)
                    if current_time - last_reroute < self.reroute_cooldown:
                        continue

                    # Check if vehicle has exceeded max reroutes
                    reroute_count = self.vehicle_reroute_count.get(vehicle_id, 0)
                    if reroute_count >= self.max_reroutes_per_vehicle:
                        continue  # Skip rerouting this vehicle
                    
                    # Vehicle needs rerouting (but only do it if rerouting is enabled)
                    if self.enable_rerouting:
                        self._last_reroute_skip_reason = None
                        success = self.reroute_vehicle(vehicle_id, current_time, blocked_edges)
                        if success:
                            self.vehicle_last_reroute_time[vehicle_id] = current_time
                            rerouted_this_step.add(vehicle_id)
                            self.unique_vehicles_rerouted.add(vehicle_id)
                            self.vehicle_reroute_count[vehicle_id] = reroute_count + 1
                        else:
                            if self._last_reroute_skip_reason == "cost":
                                reroute_skipped += 1
                            else:
                                reroute_failures += 1
                            # Debug: Print why rerouting failed (show more details)
                            if reroute_failures <= 10:
                                print(f"[WARNING] Failed to reroute vehicle {vehicle_id}")
                                print(f"   Current edge: {current_edge}")
                                print(f"   Destination edge: {destination_edge}")
                                print(f"   Route length: {len(route) if route else 0}")
                                if current_edge not in self.edge_info:
                                    print(f"   [ERROR] Current edge not in graph (graph has {len(self.edge_info)} edges)")
                                if destination_edge not in self.edge_info:
                                    print(f"   [ERROR] Destination edge not in graph")
                                if current_edge in self.edge_info and destination_edge in self.edge_info:
                                    print(f"   [INFO] Both edges in graph - pathfinding may have failed")
                                    print(f"   [INFO] Check if Dijkstra can find path from {current_edge} to {destination_edge}")
                    # In baseline mode, still track that vehicle would have needed rerouting
                    elif not self.enable_rerouting:
                        # Mark as affected but don't reroute
                        if vehicle_id in self.vehicle_metrics:
                            self.vehicle_metrics[vehicle_id]['crossing_affected'] = True
                            # Also mark in crossing metrics
                            for crossing in active_crossings:
                                if vehicle_id in self.crossing_metrics.get(crossing.crossing_id, {}).get('vehicles_affected', set()):
                                    pass  # Already tracked
                                else:
                                    if crossing.crossing_id not in self.crossing_metrics:
                                        self.crossing_metrics[crossing.crossing_id] = {
                                            'vehicles_affected': set(),
                                            'max_queue_length': 0,
                                            'total_delay': 0.0,
                                            'start_time': current_time,
                                            'duration': 0.0,
                                            'blocked_edges': crossing.edge_ids.copy()
                                        }
                                    self.crossing_metrics[crossing.crossing_id]['vehicles_affected'].add(vehicle_id)
                
            except Exception as e:
                # Vehicle might have left or have issues
                if reroute_failures < 3:
                    print(f"[WARNING] Exception checking vehicle {vehicle_id}: {e}")
                pass
        
        self.total_vehicles = len(vehicles_to_check)
        
        # Debug output every 10 seconds when there are active crossings OR if no crossings but vehicles exist
        if (active_crossings and int(current_time) % 10 == 0) or (not active_crossings and len(vehicles_to_check) > 0 and int(current_time) % 60 == 0 and current_time > 0):
            print(f"\n[{current_time:.1f}s] [ROUTING UPDATE]:")
            print(f"   Active crossings: {len(active_crossings)}")
            print(f"   Blocked edges: {len(blocked_edges)}")
            print(f"   Vehicles checked: {len(vehicles_to_check)}")
            print(f"   Vehicles needing reroute: {vehicles_need_reroute}")
            print(f"   Vehicles rerouted this step: {len(rerouted_this_step)}")
            print(f"   Reroute failures: {reroute_failures}")
            if reroute_skipped > 0:
                print(f"   Reroutes skipped (detour too costly): {reroute_skipped}")
            print(f"   Unique vehicles rerouted (total): {len(self.unique_vehicles_rerouted)}")
            if len(blocked_edges) > 0 and len(blocked_edges) <= 20:
                # Show sample of blocked edges (excluding train edges if we can identify them)
                sample_edges = list(blocked_edges)[:5]
                print(f"   Sample blocked edges: {sample_edges}")
            # Debug: Show why vehicles aren't being rerouted when crossings are active
            if active_crossings and vehicles_need_reroute == 0 and len(vehicles_to_check) > 0:
                print(f"   [DEBUG] Active crossings but no vehicles need rerouting - investigating...")
                sample_checked = 0
                for veh_id in vehicles_to_check[:10]:  # Check first 10 vehicles
                    try:
                        curr_edge, dest_edge, route = self.get_vehicle_route_info(veh_id)
                        if route:
                            route_set = set(route)
                            intersection = route_set & blocked_edges
                            if not intersection:
                                # Check original route
                                if veh_id in self.vehicle_routes:
                                    orig_route_set = set(self.vehicle_routes[veh_id])
                                    orig_intersection = orig_route_set & blocked_edges
                                    if orig_intersection:
                                        print(f"   [DEBUG] Vehicle {veh_id}: Original route intersects but current doesn't")
                                        sample_checked += 1
                                if sample_checked < 3:
                                    print(f"   [DEBUG] Vehicle {veh_id}: No intersection - current_edge={curr_edge}, route_len={len(route)}")
                                    sample_checked += 1
                            else:
                                print(f"   [DEBUG] Vehicle {veh_id}: Route DOES intersect: {list(intersection)[:3]}")
                                sample_checked += 1
                            if sample_checked >= 5:
                                break
                    except:
                        pass
                # Show what edges are blocked
                train_edges = {e for e in blocked_edges if any(t in str(e) for t in self.train_vehicles) or "rail" in str(e).lower()}
                road_edges = blocked_edges - train_edges
                print(f"   [DEBUG] Blocked edges breakdown: {len(train_edges)} train edges, {len(road_edges)} road edges")
                if len(road_edges) == 0:
                    print(f"   [DEBUG] WARNING: No road edges blocked - only train edges. Vehicles won't intersect!")
            
            # Debug: Show intersection info
            if vehicles_need_reroute > 0 and len(blocked_edges) > 0:
                # Check a sample vehicle to see route intersection
                sample_checked = 0
                for veh_id in vehicles_to_check[:5]:
                    try:
                        curr_edge, dest_edge, route = self.get_vehicle_route_info(veh_id)
                        if route:
                            route_set = set(route)
                            intersection = route_set & blocked_edges
                            if intersection:
                                print(f"   Sample: Vehicle {veh_id} route intersects blocked edges: {list(intersection)[:3]}")
                                sample_checked += 1
                                if sample_checked >= 2:
                                    break
                    except:
                        pass
            # Debug: If no crossings but vehicles exist, explain why no rerouting
            if not active_crossings and len(vehicles_to_check) > 0:
                print(f"   [INFO] No active train crossings - vehicles will only be rerouted when trains are present")
                print(f"   [INFO] Trains are scheduled at: train_1 (300s), train_2 (480s)")
            print()
    
    def get_statistics(self) -> Dict:
        """
        Get simulation statistics.
        
        Returns:
            Dictionary with statistics
        """
        # Calculate congestion and travel time metrics
        congestion_metrics = self._calculate_congestion_metrics()
        
        return {
            'total_vehicles': self.total_vehicles,
            'total_vehicles_seen': len(self.total_vehicles_seen),
            'reroute_count': self.reroute_count,  # Total reroute operations
            'unique_vehicles_rerouted': len(self.unique_vehicles_rerouted),  # Unique vehicles rerouted
            'active_crossings': len(self.active_crossings),
            'train_vehicles': len(self.train_vehicles),
            **congestion_metrics  # Add congestion metrics
        }
    
    def _calculate_congestion_metrics(self) -> Dict:
        """Calculate congestion and travel time metrics from collected data."""
        if not self.vehicle_metrics:
            return {}
        
        # Separate vehicles by category
        all_vehicles = list(self.vehicle_metrics.keys())
        rerouted_vehicles = [v for v in all_vehicles if self.vehicle_metrics[v].get('was_rerouted', False)]
        non_rerouted_vehicles = [v for v in all_vehicles if not self.vehicle_metrics[v].get('was_rerouted', False)]
        crossing_affected = [v for v in all_vehicles if self.vehicle_metrics[v].get('crossing_affected', False)]
        
        def calc_avg(vehicles, metric_key):
            if not vehicles:
                return 0.0
            values = [self.vehicle_metrics[v].get(metric_key, 0.0) for v in vehicles if v in self.vehicle_metrics]
            return sum(values) / len(values) if values else 0.0
        
        # Calculate average speeds
        def calc_avg_speed(vehicles):
            if not vehicles:
                return 0.0
            speeds = []
            for v in vehicles:
                if v in self.vehicle_metrics:
                    m = self.vehicle_metrics[v]
                    if m.get('speed_samples', 0) > 0:
                        speeds.append(m['speed_sum'] / m['speed_samples'])
            return sum(speeds) / len(speeds) if speeds else 0.0
        
        # Calculate route efficiency (actual distance / original distance)
        def calc_route_efficiency(vehicles):
            if not vehicles:
                return 1.0
            efficiencies = []
            for v in vehicles:
                if v in self.vehicle_metrics:
                    m = self.vehicle_metrics[v]
                    orig = m.get('original_route_length', 0)
                    actual = m.get('actual_distance', 0)
                    if orig > 0:
                        efficiencies.append(actual / orig if actual > 0 else 1.0)
            return sum(efficiencies) / len(efficiencies) if efficiencies else 1.0
        
        # Crossing metrics
        crossing_stats = {}
        for crossing_id, metrics in self.crossing_metrics.items():
            vehicles_affected = len(metrics.get('vehicles_affected', set()))
            if vehicles_affected > 0:
                avg_delay = metrics.get('total_delay', 0.0) / vehicles_affected
            else:
                avg_delay = 0.0
            
            crossing_stats[crossing_id] = {
                'vehicles_affected': vehicles_affected,
                'max_queue_length': metrics.get('max_queue_length', 0),
                'total_delay': metrics.get('total_delay', 0.0),
                'avg_delay_per_vehicle': avg_delay,
                'duration': metrics.get('duration', 0.0)
            }
        
        return {
            # Overall metrics
            'avg_travel_time_all': calc_avg(all_vehicles, 'travel_time'),
            'avg_delay_time_all': calc_avg(all_vehicles, 'delay_time'),
            'avg_crossing_delay_all': calc_avg(crossing_affected, 'crossing_delay'),
            'avg_speed_all': calc_avg_speed(all_vehicles),
            'route_efficiency_all': calc_route_efficiency(all_vehicles),
            
            # Rerouted vs Non-rerouted comparison
            'avg_travel_time_rerouted': calc_avg(rerouted_vehicles, 'travel_time'),
            'avg_travel_time_non_rerouted': calc_avg(non_rerouted_vehicles, 'travel_time'),
            'avg_delay_rerouted': calc_avg(rerouted_vehicles, 'delay_time'),
            'avg_delay_non_rerouted': calc_avg(non_rerouted_vehicles, 'delay_time'),
            'avg_speed_rerouted': calc_avg_speed(rerouted_vehicles),
            'avg_speed_non_rerouted': calc_avg_speed(non_rerouted_vehicles),
            'route_efficiency_rerouted': calc_route_efficiency(rerouted_vehicles),
            'route_efficiency_non_rerouted': calc_route_efficiency(non_rerouted_vehicles),
            
            # Crossing-affected vehicles
            'vehicles_affected_by_crossings': len(crossing_affected),
            'avg_crossing_delay': calc_avg(crossing_affected, 'crossing_delay'),
            
            # Crossing-specific stats
            'crossing_stats': crossing_stats,
            
            # Edge congestion (average vehicle count on crossing edges)
            'avg_edge_congestion': self._calculate_edge_congestion()
        }
    
    def _calculate_edge_congestion(self) -> float:
        """Calculate average congestion on edges near crossings."""
        if not self.edge_congestion:
            return 0.0
        
        all_counts = []
        for edge_id, counts in self.edge_congestion.items():
            if counts:
                all_counts.extend(counts)
        
        return sum(all_counts) / len(all_counts) if all_counts else 0.0
    
    def close(self):
        """Close the SUMO simulation."""
        traci.close()
        print("SUMO simulation closed")

