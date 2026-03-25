"""
Metrics Collection Module for SUMO Simulation
Handles collection, calculation, and analysis of performance metrics including LOS (Level of Service)
"""

import traci
from typing import Dict, List, Set, Tuple, Optional
from collections import defaultdict
import json


class MetricsCollector:
    """
    Collects and calculates performance metrics from SUMO simulation.
    Includes Level of Service (LOS) calculation based on traffic engineering standards.
    """
    
    def __init__(self):
        """Initialize metrics collector."""
        # Vehicle metrics storage
        self.vehicle_metrics: Dict[str, Dict] = {}
        
        # Crossing-specific metrics
        self.crossing_metrics: Dict[str, Dict] = {}
        
        # Edge congestion metrics
        self.edge_congestion: Dict[str, List[float]] = defaultdict(list)
        self.crossing_edges_tracked: Set[str] = set()
        
        # Track unique vehicles
        self.total_vehicles_seen: Set[str] = set()
        self.unique_vehicles_rerouted: Set[str] = set()
        
        # Edge information cache
        self.edge_info: Dict[str, Dict] = {}
        
    def initialize_edge_info(self):
        """Initialize edge information from SUMO network."""
        edge_ids = traci.edge.getIDList()
        for edge_id in edge_ids:
            try:
                length = traci.edge.getLength(edge_id)
                max_speed = traci.lane.getMaxSpeed(edge_id + "_0")  # Assume lane 0
                self.edge_info[edge_id] = {
                    'length': length,
                    'max_speed': max_speed,
                    'free_flow_speed': max_speed  # Free flow speed (m/s)
                }
            except:
                try:
                    # Try without lane suffix
                    max_speed = traci.edge.getMaxSpeed(edge_id)
                    self.edge_info[edge_id] = {
                        'length': traci.edge.getLength(edge_id),
                        'max_speed': max_speed,
                        'free_flow_speed': max_speed
                    }
                except:
                    pass
    
    def update_vehicle_metrics(self, current_time: float, active_crossings: List, 
                              blocked_edges: Set[str], train_vehicles: Set[str],
                              vehicle_routes: Dict[str, List[str]]):
        """
        Update metrics for all vehicles in the simulation.
        
        Args:
            current_time: Current simulation time
            active_crossings: List of active train crossings
            blocked_edges: Set of blocked edge IDs
            train_vehicles: Set of train vehicle IDs
            vehicle_routes: Dictionary of vehicle routes
        """
        all_vehicles = traci.vehicle.getIDList()
        
        for vehicle_id in all_vehicles:
            if vehicle_id in train_vehicles:
                continue
                
            try:
                # Initialize metrics if vehicle is new
                if vehicle_id not in self.vehicle_metrics:
                    route = traci.vehicle.getRoute(vehicle_id)
                    original_length = sum(
                        self.edge_info.get(e, {}).get('length', 0) 
                        for e in route if e in self.edge_info
                    )
                    
                    # Calculate free-flow travel time
                    free_flow_time = 0.0
                    for edge_id in route:
                        if edge_id in self.edge_info:
                            edge_data = self.edge_info[edge_id]
                            length = edge_data.get('length', 0)
                            free_flow_speed = edge_data.get('free_flow_speed', 15.65)  # Default 35 mph
                            if free_flow_speed > 0:
                                free_flow_time += length / free_flow_speed
                    
                    self.vehicle_metrics[vehicle_id] = {
                        'start_time': current_time,
                        'original_route_length': original_length,
                        'free_flow_travel_time': free_flow_time,
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
                        'crossing_start_time': None,
                        'edges_traveled': set()  # Track edges for density calculation
                    }
                
                metrics = self.vehicle_metrics[vehicle_id]
                
                # Check if vehicle still exists
                try:
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                    position = traci.vehicle.getLanePosition(vehicle_id)
                    distance = traci.vehicle.getDistance(vehicle_id)
                    
                    # Track edges traveled
                    if current_edge:
                        metrics['edges_traveled'].add(current_edge)
                    
                    # Update speed metrics
                    metrics['max_speed'] = max(metrics['max_speed'], speed)
                    metrics['speed_sum'] += speed
                    metrics['speed_samples'] += 1
                    
                    # Update distance (use SUMO's distance measurement)
                    metrics['actual_distance'] = distance
                    
                    # Update travel time
                    metrics['travel_time'] = current_time - metrics['start_time']
                    
                    # Fix #4: Delay = actual travel time minus free-flow (standard traffic engineering definition)
                    # Replaces speed-threshold counting which inflated delay with non-crossing stops.
                    free_flow = metrics.get('free_flow_travel_time', 0)
                    if free_flow > 0:
                        metrics['delay_time'] = max(0.0, metrics['travel_time'] - free_flow)
                    elif speed < 5.0:
                        metrics['delay_time'] += 1.0  # Fallback if free_flow not set
                    
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
    
    def update_crossing_metrics(self, current_time: float, active_crossings: List, 
                               vehicles_to_check: List[str], vehicle_routes: Dict[str, List[str]]):
        """
        Update metrics specific to train crossings.
        
        Args:
            current_time: Current simulation time
            active_crossings: List of active train crossings
            vehicles_to_check: List of vehicle IDs to check
            vehicle_routes: Dictionary of vehicle routes
        """
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
            
            # Count vehicles currently in crossing area and approaching it
            # This includes vehicles stopped at crossing gates/stops
            vehicles_in_area = 0
            for vehicle_id in vehicles_to_check:
                try:
                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                    route = traci.vehicle.getRoute(vehicle_id)
                    speed = traci.vehicle.getSpeed(vehicle_id)
                    is_affected = False
                    
                    # Check if vehicle is currently on crossing edge
                    if current_edge in crossing.edge_ids:
                        vehicles_in_area += 1
                        metrics['vehicles_affected'].add(vehicle_id)
                        is_affected = True
                        
                        # Check if vehicle is delayed (low speed) - stopped at gate
                        if speed < 5.0:
                            metrics['total_delay'] += 1.0  # 1 second delay
                    
                    # IMPORTANT: Detect vehicles stopped/slowed near crossings (at gates/stops)
                    # These vehicles are affected even if not directly on train edges
                    if not is_affected and speed < 2.0:  # Stopped or nearly stopped
                        # Check if vehicle is near crossing area
                        # Method 1: Check if vehicle's route intersects crossing edges
                        if route:
                            route_edges = set(route)
                            if route_edges & crossing.edge_ids:
                                # Vehicle is stopped and its route goes through crossing
                                metrics['vehicles_affected'].add(vehicle_id)
                                vehicles_in_area += 1
                                is_affected = True
                                metrics['total_delay'] += 1.0  # Count delay for stopped vehicle
                        
                        # Method 2: Check if vehicle is at same junction as train edges
                        # (vehicles stop at crossing gates which are at junctions)
                        if not is_affected:
                            try:
                                veh_to_junction = traci.edge.getToJunction(current_edge)
                                veh_from_junction = traci.edge.getFromJunction(current_edge)
                                
                                # Check if any train edge is at same junction
                                for train_edge in crossing.edge_ids:
                                    try:
                                        train_to = traci.edge.getToJunction(train_edge)
                                        train_from = traci.edge.getFromJunction(train_edge)
                                        
                                        # If vehicle is at same junction as train, it's affected
                                        if (veh_to_junction == train_to or veh_from_junction == train_to or
                                            veh_to_junction == train_from or veh_from_junction == train_from):
                                            metrics['vehicles_affected'].add(vehicle_id)
                                            vehicles_in_area += 1
                                            is_affected = True
                                            metrics['total_delay'] += 1.0
                                            break
                                    except:
                                        pass
                            except:
                                pass
                    
                    # Check if vehicle's current route intersects crossing (for moving vehicles)
                    if route and not is_affected:
                        route_edges = set(route)
                        if route_edges & crossing.edge_ids:
                            metrics['vehicles_affected'].add(vehicle_id)
                            is_affected = True
                    
                    # Check original route if available (important for baseline mode)
                    if vehicle_id in vehicle_routes and not is_affected:
                        original_route_edges = set(vehicle_routes[vehicle_id])
                        if original_route_edges & crossing.edge_ids:
                            metrics['vehicles_affected'].add(vehicle_id)
                            is_affected = True
                    
                    # Also check if vehicle is approaching crossing (within next 10 edges)
                    # This catches vehicles that will be affected soon
                    if route and not is_affected:
                        try:
                            if current_edge in route:
                                current_idx = route.index(current_edge)
                                # Check next 10 edges in route
                                upcoming_edges = route[current_idx:min(current_idx + 10, len(route))]
                                if set(upcoming_edges) & crossing.edge_ids:
                                    metrics['vehicles_affected'].add(vehicle_id)
                        except:
                            pass
                            
                except:
                    pass
            
            # Update max queue length
            metrics['max_queue_length'] = max(metrics['max_queue_length'], vehicles_in_area)
            metrics['duration'] = current_time - metrics['start_time']
    
    def calculate_los(self, avg_speed: float, free_flow_speed: float, 
                     delay_per_vehicle: float, density: float = None) -> str:
        """
        Calculate Level of Service (LOS) based on traffic engineering standards.
        
        LOS Criteria (for urban arterials):
        - LOS A: Speed > 90% free-flow, delay < 10s/veh, density < 7 veh/km/lane
        - LOS B: Speed 70-90%, delay 10-20s/veh, density 7-11 veh/km/lane
        - LOS C: Speed 50-70%, delay 20-35s/veh, density 11-16 veh/km/lane
        - LOS D: Speed 40-50%, delay 35-55s/veh, density 16-22 veh/km/lane
        - LOS E: Speed 30-40%, delay 55-80s/veh, density 22-28 veh/km/lane
        - LOS F: Speed < 30%, delay > 80s/veh, density > 28 veh/km/lane
        
        Args:
            avg_speed: Average speed (m/s)
            free_flow_speed: Free-flow speed (m/s)
            delay_per_vehicle: Average delay per vehicle (seconds)
            density: Vehicle density (veh/km/lane), optional
            
        Returns:
            LOS letter (A-F)
        """
        if free_flow_speed == 0:
            return "F"
        
        speed_ratio = avg_speed / free_flow_speed if free_flow_speed > 0 else 0
        
        # Determine LOS based on speed ratio and delay
        if speed_ratio >= 0.90 and delay_per_vehicle < 10:
            return "A"
        elif speed_ratio >= 0.70 and delay_per_vehicle < 20:
            return "B"
        elif speed_ratio >= 0.50 and delay_per_vehicle < 35:
            return "C"
        elif speed_ratio >= 0.40 and delay_per_vehicle < 55:
            return "D"
        elif speed_ratio >= 0.30 and delay_per_vehicle < 80:
            return "E"
        else:
            return "F"
    
    def calculate_metrics(self) -> Dict:
        """
        Calculate comprehensive performance metrics including LOS.
        
        Returns:
            Dictionary with all calculated metrics
        """
        if not self.vehicle_metrics:
            return {}
        
        # Separate vehicles by category
        all_vehicles = list(self.vehicle_metrics.keys())
        rerouted_vehicles = [v for v in all_vehicles 
                            if self.vehicle_metrics[v].get('was_rerouted', False)]
        non_rerouted_vehicles = [v for v in all_vehicles 
                                if not self.vehicle_metrics[v].get('was_rerouted', False)]
        crossing_affected = [v for v in all_vehicles 
                            if self.vehicle_metrics[v].get('crossing_affected', False)]
        
        def calc_avg(vehicles, metric_key):
            if not vehicles:
                return 0.0
            values = [self.vehicle_metrics[v].get(metric_key, 0.0) 
                     for v in vehicles if v in self.vehicle_metrics]
            return sum(values) / len(values) if values else 0.0
        
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
        
        # Calculate average free-flow speed
        def calc_avg_free_flow_speed(vehicles):
            if not vehicles:
                return 15.65  # Default 35 mph
            speeds = []
            for v in vehicles:
                if v in self.vehicle_metrics:
                    m = self.vehicle_metrics[v]
                    # Estimate from route
                    if m.get('free_flow_travel_time', 0) > 0 and m.get('original_route_length', 0) > 0:
                        estimated_speed = m['original_route_length'] / m['free_flow_travel_time']
                        speeds.append(estimated_speed)
            return sum(speeds) / len(speeds) if speeds else 15.65
        
        # Calculate metrics for each category
        avg_travel_time_all = calc_avg(all_vehicles, 'travel_time')
        avg_delay_time_all = calc_avg(all_vehicles, 'delay_time')
        avg_speed_all = calc_avg_speed(all_vehicles)
        avg_free_flow_speed_all = calc_avg_free_flow_speed(all_vehicles)
        
        # Calculate LOS for all vehicles
        delay_per_vehicle_all = avg_delay_time_all
        los_all = self.calculate_los(avg_speed_all, avg_free_flow_speed_all, delay_per_vehicle_all)
        
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
        
        # Network-wide total delay (sum of all vehicle delay_time)
        total_delay_all = sum(
            self.vehicle_metrics[v].get('delay_time', 0.0) for v in all_vehicles
        )
        # Aggregate queue length across crossings
        queue_lengths = [m['max_queue_length'] for m in crossing_stats.values()]
        queue_length_max = max(queue_lengths, default=0)
        queue_length_avg = sum(queue_lengths) / len(queue_lengths) if queue_lengths else 0.0
        
        # Calculate LOS for rerouted and non-rerouted vehicles
        avg_speed_rerouted = calc_avg_speed(rerouted_vehicles)
        avg_speed_non_rerouted = calc_avg_speed(non_rerouted_vehicles)
        avg_delay_rerouted = calc_avg(rerouted_vehicles, 'delay_time')
        avg_delay_non_rerouted = calc_avg(non_rerouted_vehicles, 'delay_time')
        
        free_flow_rerouted = calc_avg_free_flow_speed(rerouted_vehicles) if rerouted_vehicles else avg_free_flow_speed_all
        free_flow_non_rerouted = calc_avg_free_flow_speed(non_rerouted_vehicles) if non_rerouted_vehicles else avg_free_flow_speed_all
        
        los_rerouted = self.calculate_los(avg_speed_rerouted, free_flow_rerouted, avg_delay_rerouted) if rerouted_vehicles else "N/A"
        los_non_rerouted = self.calculate_los(avg_speed_non_rerouted, free_flow_non_rerouted, avg_delay_non_rerouted) if non_rerouted_vehicles else "N/A"
        
        return {
            # Overall metrics
            'avg_travel_time_all': avg_travel_time_all,
            'avg_delay_time_all': avg_delay_time_all,
            'avg_crossing_delay_all': calc_avg(crossing_affected, 'crossing_delay'),
            'avg_speed_all': avg_speed_all,
            'avg_free_flow_speed_all': avg_free_flow_speed_all,
            'speed_ratio_all': avg_speed_all / avg_free_flow_speed_all if avg_free_flow_speed_all > 0 else 0,
            'route_efficiency_all': calc_route_efficiency(all_vehicles),
            'los_all': los_all,
            'delay_per_vehicle_all': delay_per_vehicle_all,
            'total_delay_all': total_delay_all,
            'queue_length_max': queue_length_max,
            'queue_length_avg': queue_length_avg,
            
            # Rerouted vs Non-rerouted comparison
            'avg_travel_time_rerouted': calc_avg(rerouted_vehicles, 'travel_time'),
            'avg_travel_time_non_rerouted': calc_avg(non_rerouted_vehicles, 'travel_time'),
            'avg_delay_rerouted': avg_delay_rerouted,
            'avg_delay_non_rerouted': avg_delay_non_rerouted,
            'avg_speed_rerouted': avg_speed_rerouted,
            'avg_speed_non_rerouted': avg_speed_non_rerouted,
            'speed_ratio_rerouted': avg_speed_rerouted / free_flow_rerouted if free_flow_rerouted > 0 else 0,
            'speed_ratio_non_rerouted': avg_speed_non_rerouted / free_flow_non_rerouted if free_flow_non_rerouted > 0 else 0,
            'route_efficiency_rerouted': calc_route_efficiency(rerouted_vehicles),
            'route_efficiency_non_rerouted': calc_route_efficiency(non_rerouted_vehicles),
            'los_rerouted': los_rerouted,
            'los_non_rerouted': los_non_rerouted,
            
            # Crossing-affected vehicles
            'vehicles_affected_by_crossings': len(crossing_affected),
            'avg_crossing_delay': calc_avg(crossing_affected, 'crossing_delay'),
            
            # Crossing-specific stats
            'crossing_stats': crossing_stats,
            
            # Edge congestion
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
    
    def get_statistics(self) -> Dict:
        """
        Get comprehensive simulation statistics including LOS.
        
        Returns:
            Dictionary with all statistics
        """
        congestion_metrics = self.calculate_metrics()
        
        return {
            'total_vehicles': len([v for v in self.vehicle_metrics.keys()]),
            'total_vehicles_seen': len(self.total_vehicles_seen),
            'unique_vehicles_rerouted': len(self.unique_vehicles_rerouted),
            'active_crossings': len(self.crossing_metrics),
            **congestion_metrics
        }
    
    def save_metrics(self, filename: str, simulation_time: float, real_time: float, mode: str = "baseline"):
        """
        Save metrics to JSON file.
        
        Args:
            filename: Output filename
            simulation_time: Simulation duration
            real_time: Real time elapsed
            mode: Simulation mode ('baseline' or 'rerouting')
        """
        stats = self.get_statistics()
        
        output = {
            'mode': mode,
            'simulation_time': simulation_time,
            'real_time': real_time,
            'statistics': stats,
            'los_summary': {
                'overall_los': stats.get('los_all', 'N/A'),
                'rerouted_los': stats.get('los_rerouted', 'N/A'),
                'non_rerouted_los': stats.get('los_non_rerouted', 'N/A')
            }
        }
        
        # Convert sets to lists for JSON serialization
        for crossing_id, crossing_data in stats.get('crossing_stats', {}).items():
            if isinstance(crossing_data.get('vehicles_affected'), set):
                crossing_data['vehicles_affected'] = len(crossing_data['vehicles_affected'])
        
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)
        
        return output

