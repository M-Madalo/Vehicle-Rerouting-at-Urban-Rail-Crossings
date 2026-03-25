"""
Visualize the Dijkstra graph used for rerouting.

This script creates a visual representation of the network graph that Dijkstra's
algorithm uses for pathfinding and rerouting around train crossings.
"""

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from typing import Dict, List, Set, Optional, Tuple
from collections import defaultdict
import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from enhanced_dijkstra import Edge, EnhancedDijkstra
from sumo_controller import SUMOController
import traci


def extract_graph_from_controller(controller: SUMOController, include_all_edges: bool = False) -> Tuple[nx.DiGraph, Dict]:
    """
    Extract the Dijkstra graph from SUMOController and convert to NetworkX format.
    
    Args:
        controller: SUMOController instance with initialized graph
        include_all_edges: If True, include all edges from edge_info, not just those with connections
        
    Returns:
        Tuple of (NetworkX DiGraph, edge_attributes_dict)
    """
    G = nx.DiGraph()
    edge_attrs = {}
    
    # Add all nodes (edges in the original graph)
    if include_all_edges:
        # Include ALL edges from edge_info
        nodes_to_add = set(controller.edge_info.keys())
        print(f"    [DEBUG] Including all {len(nodes_to_add)} edges from edge_info")
    else:
        # Only include edges that are in the Dijkstra graph (have connections)
        nodes_to_add = controller.dijkstra.nodes
        print(f"    [DEBUG] Including only {len(nodes_to_add)} edges with connections")
    
    for node_id in nodes_to_add:
        G.add_node(node_id)
        edge_info = controller.edge_info.get(node_id, {})
        edge_attrs[node_id] = {
            'length': edge_info.get('length', 0),
            'max_speed': edge_info.get('max_speed', 0),
            'is_blocked': False,
            'congestion_factor': 1.0,
            'vehicle_count': 0,
            'has_connections': node_id in controller.dijkstra.nodes
        }
    
    # Add edges (connections between edges)
    for from_node, edge_list in controller.dijkstra.graph.items():
        for edge in edge_list:
            to_node = edge.to_node
            # Only add edge if both nodes are in the graph
            if from_node in G.nodes() and to_node in G.nodes():
                G.add_edge(from_node, to_node)
                
                # Store edge attributes
                edge_key = (from_node, to_node)
                edge_attrs[edge_key] = {
                    'edge_id': edge.edge_id,
                    'length': edge.length,
                    'max_speed': edge.max_speed,
                    'base_weight': edge.base_weight,
                    'current_weight': edge.current_weight,
                    'is_blocked': edge.is_blocked,
                    'congestion_factor': edge.congestion_factor,
                    'vehicle_count': edge.vehicle_count
                }
    
    return G, edge_attrs


def get_blocked_edges(controller: SUMOController) -> Set[str]:
    """Get currently blocked edges from active train crossings."""
    blocked = set()
    # active_crossings is a dictionary, not a list
    if isinstance(controller.active_crossings, dict):
        for crossing_id, crossing in controller.active_crossings.items():
            if hasattr(crossing, 'edge_ids'):
                blocked.update(crossing.edge_ids)
            elif isinstance(crossing, dict):
                # Handle if it's stored as a dict
                blocked.update(crossing.get('edge_ids', set()))
    elif isinstance(controller.active_crossings, list):
        for crossing in controller.active_crossings:
            if hasattr(crossing, 'edge_ids'):
                blocked.update(crossing.edge_ids)
    return blocked


def visualize_graph(G: nx.DiGraph, edge_attrs: Dict, 
                   blocked_edges: Optional[Set[str]] = None,
                   highlight_path: Optional[List[str]] = None,
                   title: str = "Dijkstra Rerouting Graph",
                   output_file: str = "dijkstra_graph.png",
                   layout: str = "spring",
                   show_all_edges: bool = False):
    """
    Visualize the Dijkstra graph with various highlighting options.
    
    Args:
        G: NetworkX directed graph
        edge_attrs: Dictionary of edge attributes
        blocked_edges: Set of blocked edge IDs to highlight
        highlight_path: List of edge IDs representing a path to highlight
        title: Title for the plot
        output_file: Output filename
        layout: Layout algorithm ('spring', 'circular', 'kamada_kawai', 'planar')
    """
    if len(G.nodes()) == 0:
        print("[WARNING] Graph is empty - cannot visualize")
        return
    
    # Choose layout - adjust parameters for large graphs
    num_nodes = len(G.nodes())
    if num_nodes > 100:
        # For large graphs, use spring layout with more spacing
        k_value = 2.0  # Increased spacing
        iterations = 100  # More iterations for better layout
    else:
        k_value = 1.0
        iterations = 50
    
    if layout == "spring":
        pos = nx.spring_layout(G, k=k_value, iterations=iterations, seed=42)
    elif layout == "circular":
        pos = nx.circular_layout(G)
    elif layout == "kamada_kawai":
        try:
            pos = nx.kamada_kawai_layout(G)
        except:
            pos = nx.spring_layout(G, k=k_value, iterations=iterations, seed=42)
    elif layout == "planar":
        try:
            pos = nx.planar_layout(G)
        except:
            pos = nx.spring_layout(G, k=k_value, iterations=iterations, seed=42)
    else:
        pos = nx.spring_layout(G, k=k_value, iterations=iterations, seed=42)
    
    # Create figure - larger for big graphs
    num_nodes = len(G.nodes())
    if num_nodes > 100:
        figsize = (20, 16)  # Larger figure for many nodes
    else:
        figsize = (16, 12)
    fig, ax = plt.subplots(figsize=figsize)
    
    # Determine node colors and sizes
    node_colors = []
    node_sizes = []
    for node in G.nodes():
        node_attrs = edge_attrs.get(node, {})
        has_connections = node_attrs.get('has_connections', True)
        
        if blocked_edges and node in blocked_edges:
            node_colors.append('#ff0000')  # Red for blocked
            node_sizes.append(150)  # Larger for blocked
        elif highlight_path and node in highlight_path:
            node_colors.append('#00ff00')  # Green for path
            node_sizes.append(150)  # Larger for path
        elif show_all_edges and not has_connections:
            node_colors.append('#888888')  # Darker gray for better visibility
            node_sizes.append(40)  # Smaller but still visible for isolated edges
        else:
            node_colors.append('#1f77b4')  # Blue for normal
            node_sizes.append(400)  # Much larger for better visibility
    
    # Determine edge colors and widths
    edge_colors = []
    edge_widths = []
    for edge in G.edges():
        edge_key = edge
        attrs = edge_attrs.get(edge_key, {})
        
        if highlight_path and edge[0] in highlight_path and edge[1] in highlight_path:
            # Check if this edge is part of the path
            idx = highlight_path.index(edge[0]) if edge[0] in highlight_path else -1
            if idx >= 0 and idx < len(highlight_path) - 1 and highlight_path[idx + 1] == edge[1]:
                edge_colors.append('#00ff00')  # Green for path edges
                edge_widths.append(3.0)
            else:
                if attrs.get('is_blocked', False):
                    edge_colors.append('#ff0000')  # Red for blocked
                    edge_widths.append(2.0)
                else:
                    edge_colors.append('#000000')  # Black for normal (better visibility)
                    edge_widths.append(1.0)  # Thicker for visibility
        elif attrs.get('is_blocked', False):
            edge_colors.append('#ff0000')  # Red for blocked
            edge_widths.append(2.5)  # Thicker for blocked edges
        else:
            edge_colors.append('#000000')  # Black for normal (better visibility)
            edge_widths.append(1.0)  # Thicker for visibility
    
    # Draw edges
    nx.draw_networkx_edges(G, pos, ax=ax, 
                           edge_color=edge_colors,
                           width=edge_widths,
                           alpha=0.8,  # More opaque for better visibility
                           arrows=True,
                           arrowsize=15,  # Larger arrows
                           arrowstyle='->')
    
    # Draw nodes - handle large graphs better
    if show_all_edges and num_nodes > 50:
        # For large graphs, draw isolated nodes separately for clarity
        node_list = list(G.nodes())
        isolated_nodes = [node for node in node_list 
                         if not edge_attrs.get(node, {}).get('has_connections', False)]
        connected_nodes = [node for node in node_list 
                          if edge_attrs.get(node, {}).get('has_connections', False)]
        
        # Draw isolated nodes first (behind) - more transparent
        if isolated_nodes:
            isolated_colors = [node_colors[node_list.index(n)] for n in isolated_nodes]
            isolated_sizes = [node_sizes[node_list.index(n)] for n in isolated_nodes]
            nx.draw_networkx_nodes(G, pos, ax=ax, nodelist=isolated_nodes,
                                  node_color=isolated_colors,
                                  node_size=isolated_sizes,
                                  alpha=0.5)  # More transparent for background
        
        # Draw connected nodes on top - more opaque and detailed
        if connected_nodes:
            connected_colors = [node_colors[node_list.index(n)] for n in connected_nodes]
            connected_sizes = [node_sizes[node_list.index(n)] for n in connected_nodes]
            nx.draw_networkx_nodes(G, pos, ax=ax, nodelist=connected_nodes,
                                  node_color=connected_colors,
                                  node_size=connected_sizes,
                                  alpha=1.0,  # Fully opaque
                                  edgecolors='darkblue',  # Dark blue outline
                                  linewidths=2.0)  # Thicker outline for definition
    else:
        # Draw all nodes normally for small graphs
        # Check if these are connected nodes (blue) and add outline
        if any(edge_attrs.get(node, {}).get('has_connections', False) for node in G.nodes()):
            connected_nodes_small = [node for node in G.nodes() 
                                     if edge_attrs.get(node, {}).get('has_connections', False)]
            other_nodes_small = [node for node in G.nodes() 
                                if not edge_attrs.get(node, {}).get('has_connections', False)]
            
            if connected_nodes_small:
                connected_colors_small = [node_colors[list(G.nodes()).index(n)] for n in connected_nodes_small]
                connected_sizes_small = [node_sizes[list(G.nodes()).index(n)] for n in connected_nodes_small]
                nx.draw_networkx_nodes(G, pos, ax=ax, nodelist=connected_nodes_small,
                                      node_color=connected_colors_small,
                                      node_size=connected_sizes_small,
                                      alpha=1.0,
                                      edgecolors='darkblue',
                                      linewidths=2.0)
            if other_nodes_small:
                other_colors_small = [node_colors[list(G.nodes()).index(n)] for n in other_nodes_small]
                other_sizes_small = [node_sizes[list(G.nodes()).index(n)] for n in other_nodes_small]
                nx.draw_networkx_nodes(G, pos, ax=ax, nodelist=other_nodes_small,
                                      node_color=other_colors_small,
                                      node_size=other_sizes_small,
                                      alpha=0.8)
        else:
            nx.draw_networkx_nodes(G, pos, ax=ax,
                                  node_color=node_colors,
                                  node_size=node_sizes,
                                  alpha=0.8)
    
    # Draw labels (only for important nodes to avoid clutter)
    if len(G.nodes()) <= 50:
        # Show all labels if graph is small
        labels = {node: node[:20] + '...' if len(node) > 20 else node 
                 for node in G.nodes()}
        nx.draw_networkx_labels(G, pos, labels, ax=ax, font_size=10, font_weight='bold')
    else:
        # For large graphs, show labels for connected nodes (blue nodes) and important nodes
        labels = {}
        # Always show labels for nodes with connections (routable edges) - these are the important blue nodes
        nodes_with_connections = [node for node in G.nodes() 
                                 if edge_attrs.get(node, {}).get('has_connections', False)]
        for node in nodes_with_connections:
            labels[node] = node[:20] + '...' if len(node) > 20 else node
        
        # Always show labels for blocked and path nodes
        if blocked_edges:
            for node in blocked_edges:
                if node in G.nodes():
                    labels[node] = node[:20] + '...' if len(node) > 20 else node
        if highlight_path:
            for node in highlight_path:
                if node in G.nodes():
                    labels[node] = node[:20] + '...' if len(node) > 20 else node
        if labels:
            nx.draw_networkx_labels(G, pos, labels, ax=ax, font_size=12, 
                                   font_weight='bold',
                                   bbox=dict(boxstyle='round,pad=0.5', 
                                           facecolor='white', alpha=0.9,
                                           edgecolor='darkblue', linewidth=1.5))
    
    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#1f77b4', label='Normal Edge (with connections)'),
        Patch(facecolor='#ff0000', label='Blocked Edge (Train Crossing)'),
    ]
    if show_all_edges:
        legend_elements.append(Patch(facecolor='#cccccc', label='Edge (no connections)'))
    if highlight_path:
        legend_elements.append(Patch(facecolor='#00ff00', label='Dijkstra Path'))
    ax.legend(handles=legend_elements, loc='upper right')
    
    # Set title and labels
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    
    # Count isolated nodes if showing all edges
    isolated_count = 0
    if show_all_edges:
        isolated_count = sum(1 for node in G.nodes() 
                            if not edge_attrs.get(node, {}).get('has_connections', False))
    
    stats_text = f'Nodes: {len(G.nodes())}\nEdges: {len(G.edges())}'
    if show_all_edges and isolated_count > 0:
        stats_text += f'\nIsolated: {isolated_count}'
    
    ax.text(0.02, 0.98, 
           stats_text,
           transform=ax.transAxes,
           fontsize=10,
           verticalalignment='top',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    ax.axis('off')
    plt.tight_layout()
    
    # Save figure
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"[OK] Graph visualization saved to {output_file}")
    print(f"  Nodes: {len(G.nodes())}")
    print(f"  Edges: {len(G.edges())}")
    if blocked_edges:
        print(f"  Blocked edges: {len(blocked_edges)}")
    if highlight_path:
        print(f"  Path length: {len(highlight_path)} edges")
    
    plt.close()


def visualize_from_running_simulation(sumo_config: str, 
                                      output_file: str = "dijkstra_graph.png",
                                      highlight_path: Optional[List[str]] = None,
                                      layout: str = "spring",
                                      show_all_edges: bool = False):
    """
    Visualize the graph from a running SUMO simulation.
    
    Args:
        sumo_config: Path to SUMO config file
        output_file: Output filename
        highlight_path: Optional path to highlight
        layout: Graph layout algorithm
    """
    from enhanced_dijkstra import EnhancedDijkstra
    
    print("Initializing SUMO simulation...")
    controller = SUMOController(sumo_config, EnhancedDijkstra({}), enable_rerouting=True)
    
    # Use non-GUI mode for faster execution (no GUI window)
    controller.sumo_binary = "sumo"  # Use non-GUI version
    
    controller.start_simulation()
    
    # Run simulation steps to build the graph and wait for train crossings
    print("Building graph from vehicle routes...")
    print("  Running simulation steps to populate graph...")
    print("  Waiting for train crossings to occur (trains at 300s and 480s)...")
    
    # Run until train crossings occur (trains at 300s and 480s)
    # Run up to 3500 steps (350 seconds) to catch first train, or until blockage
    max_steps = 3500  # Enough to reach 300s train
    steps_with_blockage = 0
    blockage_detected = False
    target_time = 310.0  # Run until just after first train (300s + 10s buffer)
    
    print(f"  Running simulation until {target_time:.1f}s (or until blockage detected)...")
    
    # Track rerouting events
    rerouting_captured = False
    captured_vehicle_id = None
    captured_current_edge = None
    captured_destination_edge = None
    captured_blocked_edges = set()
    captured_path = None
    captured_time = 0.0
    
    for step in range(max_steps):
        current_time = step * 0.1
        traci.simulationStep()
        controller.run_step(current_time)
        
        # Check for active train crossings
        active_crossings = controller.detect_train_crossings(current_time)
        if active_crossings:
            blockage_detected = True
            steps_with_blockage += 1
            blocked_edges_count = sum(len(crossing.edge_ids) for crossing in active_crossings)
            print(f"  [BLOCKAGE DETECTED] Step {step} ({current_time:.1f}s): {len(active_crossings)} active crossings, {blocked_edges_count} blocked edges")
            
            # Try to capture a rerouting event
            if not rerouting_captured:
                # Get all blocked edges
                all_blocked = set()
                for crossing in active_crossings:
                    all_blocked.update(crossing.edge_ids)
                
                # Find a vehicle that was rerouted
                for vehicle_id in controller.unique_vehicles_rerouted:
                    if vehicle_id in controller.vehicle_routes:
                        route = controller.vehicle_routes[vehicle_id]
                        if len(route) > 0:
                            # Check if vehicle's route intersects blocked edges
                            if set(route) & all_blocked:
                                # This vehicle was rerouted around blocked edges
                                try:
                                    current_edge = traci.vehicle.getRoadID(vehicle_id)
                                    destination_edge = route[-1] if route else current_edge
                                    captured_vehicle_id = vehicle_id
                                    captured_current_edge = current_edge
                                    captured_destination_edge = destination_edge
                                    captured_blocked_edges = all_blocked
                                    captured_path = route
                                    captured_time = current_time
                                    rerouting_captured = True
                                    print(f"  [REROUTING CAPTURED] Vehicle {vehicle_id} at {current_time:.1f}s")
                                    print(f"    Start: {current_edge}, End: {destination_edge}")
                                    print(f"    Path: {route}")
                                    break
                                except:
                                    pass
            
            # Continue for a few more steps to ensure blockage is stable
            if steps_with_blockage >= 5:
                print(f"  Blockage confirmed - stopping at step {step} ({current_time:.1f}s)")
                break
        
        # Stop if we've reached target time
        if current_time >= target_time:
            if not blockage_detected:
                print(f"  [INFO] Reached {target_time:.1f}s but no blockage detected yet")
                print(f"  [INFO] Checking one more time...")
                # One final check
                active_crossings = controller.detect_train_crossings(current_time)
                if active_crossings:
                    blockage_detected = True
                    blocked_edges_count = sum(len(crossing.edge_ids) for crossing in active_crossings)
                    print(f"  [BLOCKAGE FOUND] {len(active_crossings)} active crossings, {blocked_edges_count} blocked edges")
            break
        
        if step % 500 == 0 and step > 0:
            vehicle_count = len(traci.vehicle.getIDList())
            graph_size = sum(len(edges) for edges in controller.dijkstra.graph.values())
            active_crossings = controller.detect_train_crossings(current_time)
            crossings_count = len(active_crossings)
            print(f"  Step {step} ({current_time:.1f}s): {vehicle_count} vehicles, {graph_size} connections, {crossings_count} crossings")
    
    if not blockage_detected:
        print(f"  [WARNING] No train crossings detected by {target_time:.1f}s")
        print(f"  [INFO] Trains are scheduled at 300s and 480s")
        print(f"  [INFO] Visualization will show graph without blocked edges")
    
    # Extract graph
    print("\nExtracting graph structure...")
    print(f"  Total edges in network (edge_info): {len(controller.edge_info)}")
    print(f"  Edges in Dijkstra graph (with connections): {len(controller.dijkstra.nodes)}")
    print(f"  Using --all-edges flag: {show_all_edges}")
    
    G, edge_attrs = extract_graph_from_controller(controller, include_all_edges=show_all_edges)
    
    if show_all_edges:
        print(f"  Extracted {len(G.nodes())} nodes (all edges from network) and {len(G.edges())} connections")
        isolated_count = sum(1 for node in G.nodes() if not edge_attrs.get(node, {}).get('has_connections', False))
        print(f"  Isolated edges (no connections): {isolated_count}")
    else:
        print(f"  Extracted {len(G.nodes())} nodes (edges with connections only) and {len(G.edges())} edges")
    
    if len(G.nodes()) == 0:
        print("\n[WARNING] Graph is empty - trying to build more connections...")
        # Try building connections dynamically
        connections_added = controller._build_connections_from_vehicles()
        if connections_added > 0:
            print(f"  Built {connections_added} additional connections")
            G, edge_attrs = extract_graph_from_controller(controller, include_all_edges=show_all_edges)
            print(f"  Now have {len(G.nodes())} nodes and {len(G.edges())} edges")
        
        if len(G.nodes()) == 0:
            print("\n[ERROR] Graph is still empty - no connections found!")
            print("  This may happen if:")
            print("  1. No vehicles have appeared yet")
            print("  2. Vehicle routes don't contain valid edge connections")
            traci.close()
            return
    
    # Get blocked edges
    print("\nChecking for blocked edges...")
    blocked_edges = get_blocked_edges(controller)
    if blocked_edges:
        print(f"  Found {len(blocked_edges)} blocked edges:")
        for i, edge_id in enumerate(list(blocked_edges)[:10]):  # Show first 10
            print(f"    - {edge_id}")
        if len(blocked_edges) > 10:
            print(f"    ... and {len(blocked_edges) - 10} more")
        
        # Also check which nodes in the graph are blocked
        blocked_nodes_in_graph = [node for node in G.nodes() if node in blocked_edges]
        print(f"  Blocked nodes in graph: {len(blocked_nodes_in_graph)}")
        if blocked_nodes_in_graph:
            print(f"    Blocked nodes: {', '.join(blocked_nodes_in_graph[:5])}")
            if len(blocked_nodes_in_graph) > 5:
                print(f"    ... and {len(blocked_nodes_in_graph) - 5} more")
    else:
        print("  No blocked edges (no active train crossings)")
        print("  [INFO] To see blocked edges, run simulation until trains appear (300s or 480s)")
    
    # Visualize - use rerouting scenario if captured, otherwise use basic visualization
    print("\nCreating visualization...")
    print(f"  Using {layout} layout algorithm...")
    if show_all_edges:
        print(f"  Showing all edges (including isolated ones)")
    
    if rerouting_captured and captured_path:
        print(f"\n  [INFO] Visualizing REROUTING SCENARIO:")
        print(f"    Vehicle: {captured_vehicle_id}")
        print(f"    Time: {captured_time:.1f}s")
        print(f"    Start: {captured_current_edge}")
        print(f"    End: {captured_destination_edge}")
        print(f"    Path: {captured_path}")
        print(f"    Blocked: {len(captured_blocked_edges)} edges")
        
        # Use the rerouting visualization with all details
        visualize_rerouting_event(
            controller=controller,
            vehicle_id=captured_vehicle_id,
            current_edge=captured_current_edge,
            destination_edge=captured_destination_edge,
            blocked_edges=captured_blocked_edges,
            new_path=captured_path,
            current_time=captured_time,
            output_file=output_file,
            max_visualizations=1,  # Just this one
            show_possible_routes=True
        )
    else:
        # Use basic visualization
        print("  [INFO] No rerouting event captured - showing basic graph")
        visualize_graph(G, edge_attrs, blocked_edges, highlight_path,
                       title="Dijkstra Rerouting Graph\n(Edge-to-Edge Network)" + 
                             ("\n(All Edges)" if show_all_edges else ""),
                       output_file=output_file,
                       layout=layout,
                       show_all_edges=show_all_edges)
    
    # Close SUMO
    traci.close()
    print("[OK] Visualization complete!")


def visualize_from_saved_graph(graph_data: Dict, 
                               output_file: str = "dijkstra_graph.png"):
    """
    Visualize a graph from saved data.
    
    Args:
        graph_data: Dictionary containing graph structure
        output_file: Output filename
    """
    G = nx.DiGraph()
    
    # Reconstruct graph from saved data
    if 'nodes' in graph_data and 'edges' in graph_data:
        G.add_nodes_from(graph_data['nodes'])
        G.add_edges_from(graph_data['edges'])
    
    # Create dummy edge_attrs
    edge_attrs = {}
    
    visualize_graph(G, edge_attrs, 
                   title="Dijkstra Rerouting Graph\n(From Saved Data)",
                   output_file=output_file)


def find_possible_routes(G: nx.DiGraph, start_node: str, end_node: str, 
                         blocked_edges: Set[str], max_paths: int = 5, 
                         max_path_length: int = 20) -> List[List[str]]:
    """
    Find multiple possible routes from start to end, avoiding blocked edges.
    
    Args:
        G: NetworkX directed graph
        start_node: Starting node
        end_node: Destination node
        blocked_edges: Set of blocked edge IDs to avoid
        max_paths: Maximum number of paths to find
        max_path_length: Maximum path length to consider
        
    Returns:
        List of possible paths (each path is a list of node IDs)
    """
    if start_node not in G or end_node not in G:
        return []
    
    possible_paths = []
    
    # Use DFS to find multiple paths
    def dfs_paths(current: str, target: str, visited: Set[str], path: List[str], depth: int):
        if depth > max_path_length:
            return
        
        if current == target:
            possible_paths.append(path.copy())
            return
        
        if len(possible_paths) >= max_paths:
            return
        
        visited.add(current)
        
        # Explore neighbors
        for neighbor in G.successors(current):
            # Skip if blocked
            if neighbor in blocked_edges:
                continue
            
            # Skip if already visited (to avoid cycles)
            if neighbor not in visited:
                path.append(neighbor)
                dfs_paths(neighbor, target, visited, path, depth + 1)
                path.pop()
        
        visited.remove(current)
    
    # Start DFS from start node
    try:
        dfs_paths(start_node, end_node, set(), [start_node], 0)
    except:
        pass
    
    # Also try using NetworkX's all_simple_paths (if available and graph is small enough)
    if len(G.nodes()) < 100:
        try:
            # Get all simple paths (no cycles)
            nx_paths = list(nx.all_simple_paths(G, start_node, end_node, cutoff=max_path_length))
            
            # Filter out paths that go through blocked edges
            for path in nx_paths:
                if not any(node in blocked_edges for node in path):
                    if path not in possible_paths:
                        possible_paths.append(path)
                    if len(possible_paths) >= max_paths:
                        break
        except:
            pass
    
    # Sort by path length (shorter paths first)
    possible_paths.sort(key=len)
    
    return possible_paths[:max_paths]


def visualize_rerouting_event(controller: SUMOController,
                               vehicle_id: str,
                               current_edge: str,
                               destination_edge: str,
                               blocked_edges: Set[str],
                               new_path: List[str],
                               current_time: float,
                               output_file: Optional[str] = None,
                               max_visualizations: int = 5,
                               show_possible_routes: bool = True):
    """
    Visualize the Dijkstra graph at the moment of rerouting.
    
    This function creates a visualization showing:
    - The graph structure (nodes and edges)
    - Blocked edges in red (train crossings)
    - The new path found by Dijkstra in bright green (chosen path)
    - Alternative possible routes in different colors
    - Start node (current edge) and end node (destination edge) highlighted
    
    Args:
        controller: SUMOController instance
        vehicle_id: ID of vehicle being rerouted
        current_edge: Current edge (start node)
        destination_edge: Destination edge (end node)
        blocked_edges: Set of blocked edge IDs
        new_path: The new path found by Dijkstra
        current_time: Current simulation time
        output_file: Output filename (auto-generated if None)
        max_visualizations: Maximum number of visualizations to create (to avoid too many files)
        show_possible_routes: If True, find and display alternative possible routes
    """
    # Track how many visualizations we've created
    if not hasattr(visualize_rerouting_event, 'count'):
        visualize_rerouting_event.count = 0
    
    visualize_rerouting_event.count += 1
    
    # Limit number of visualizations
    if visualize_rerouting_event.count > max_visualizations:
        return
    
    # Generate output filename if not provided
    if output_file is None:
        output_file = f"dijkstra_rerouting_{vehicle_id}_t{current_time:.1f}s.png"
    
    try:
        # Extract graph
        G, edge_attrs = extract_graph_from_controller(controller, include_all_edges=False)
        
        if len(G.nodes()) == 0:
            print(f"[WARNING] Cannot visualize rerouting for {vehicle_id}: graph is empty")
            return
        
        # Create a subgraph focusing on relevant nodes
        # Include: start, end, blocked edges, and path nodes
        relevant_nodes = {current_edge, destination_edge}
        relevant_nodes.update(blocked_edges)
        if new_path:
            relevant_nodes.update(new_path)
        
        # Also include neighbors of relevant nodes (1-hop neighborhood) for context
        for node in list(relevant_nodes):
            if node in G:
                relevant_nodes.update(G.predecessors(node))
                relevant_nodes.update(G.successors(node))
        
        # Create subgraph with relevant nodes
        subgraph_nodes = [n for n in G.nodes() if n in relevant_nodes]
        if len(subgraph_nodes) > 0:
            G_sub = G.subgraph(subgraph_nodes).copy()
        else:
            G_sub = G
        
        # Find possible alternative routes if requested
        possible_routes = []
        if show_possible_routes and current_edge in G_sub and destination_edge in G_sub:
            try:
                print(f"  [DEBUG] Finding alternative routes from {current_edge} to {destination_edge}")
                print(f"  [DEBUG] Graph has {len(G_sub.nodes())} nodes, {len(G_sub.edges())} edges")
                print(f"  [DEBUG] Blocked edges: {len(blocked_edges)}")
                print(f"  [DEBUG] Chosen path: {new_path}")
                
                # Find ALL possible routes (including the chosen one initially)
                all_routes = find_possible_routes(
                    G_sub, current_edge, destination_edge, blocked_edges, 
                    max_paths=10, max_path_length=20  # Increase to find more routes
                )
                
                print(f"  [DEBUG] Found {len(all_routes)} total routes")
                
                # Remove the chosen path from possible routes
                if new_path:
                    # Check if new_path is in all_routes (might be slightly different format)
                    routes_to_remove = []
                    for route in all_routes:
                        # Check if route matches new_path (allowing for start node differences)
                        if route == new_path or route[1:] == new_path or route == new_path[1:]:
                            routes_to_remove.append(route)
                        # Also check if they're very similar
                        elif len(set(route) & set(new_path)) >= min(len(route), len(new_path)) * 0.8:
                            routes_to_remove.append(route)
                    
                    for route in routes_to_remove:
                        if route in all_routes:
                            all_routes.remove(route)
                
                possible_routes = all_routes[:5]  # Take up to 5 alternative routes
                
                print(f"  [DEBUG] After removing chosen path: {len(possible_routes)} alternative routes")
                for i, route in enumerate(possible_routes):
                    print(f"    Alternative Route {i+1}: {len(route)} edges - {route}")
                
                # If no alternative routes found, try to find routes that go through different nodes
                if len(possible_routes) == 0 and new_path:
                    print(f"  [DEBUG] No alternatives found, trying to find routes avoiding chosen path nodes...")
                    # Try finding routes that avoid nodes in the chosen path (except start/end)
                    avoid_nodes = set(new_path[1:-1]) if len(new_path) > 2 else set()
                    if avoid_nodes:
                        alt_routes = find_possible_routes(
                            G_sub, current_edge, destination_edge, 
                            blocked_edges | avoid_nodes,  # Avoid both blocked and chosen path nodes
                            max_paths=5, max_path_length=25
                        )
                        possible_routes = alt_routes[:5]
                        print(f"  [DEBUG] Found {len(possible_routes)} routes avoiding chosen path nodes")
                
            except Exception as e:
                print(f"  [ERROR] Could not find alternative routes: {e}")
                import traceback
                traceback.print_exc()
        
        # Prepare title matching image style
        title = f"Dijkstra Rerouting Graph (Edge-to-Edge Network)\nVehicle: {vehicle_id} | Time: {current_time:.1f}s"
        if new_path:
            title += f" | Path: {len(new_path)} edges"
        if blocked_edges:
            title += f" | Blocked: {len(blocked_edges)} edges"
        if possible_routes:
            title += f" | Alternatives: {len(possible_routes)}"
        
        # Debug: Print path information
        print(f"  [DEBUG] Visualizing rerouting scenario:")
        print(f"    Start: {current_edge}")
        print(f"    End: {destination_edge}")
        print(f"    Dijkstra path: {new_path}")
        print(f"    Path length: {len(new_path) if new_path else 0} edges")
        print(f"    Blocked edges: {len(blocked_edges)}")
        print(f"    Alternative routes: {len(possible_routes)}")
        
        # Create custom visualization with start/end node highlighting and possible routes
        visualize_graph_with_start_end(
            G_sub, 
            edge_attrs,
            blocked_edges=blocked_edges,
            highlight_path=new_path if new_path else None,
            possible_routes=possible_routes if show_possible_routes else [],
            start_node=current_edge,
            end_node=destination_edge,
            title=title,
            output_file=output_file,
            layout="spring"
        )
        
        print(f"[VISUALIZATION] Saved rerouting graph to {output_file}")
        print(f"  Vehicle: {vehicle_id}, Time: {current_time:.1f}s")
        print(f"  Blocked edges: {len(blocked_edges)}, Path length: {len(new_path) if new_path else 0}")
        
    except Exception as e:
        print(f"[WARNING] Failed to visualize rerouting for {vehicle_id}: {e}")
        import traceback
        traceback.print_exc()


def visualize_graph_with_start_end(G: nx.DiGraph, edge_attrs: Dict, 
                                   blocked_edges: Optional[Set[str]] = None,
                                   highlight_path: Optional[List[str]] = None,
                                   possible_routes: Optional[List[List[str]]] = None,
                                   start_node: Optional[str] = None,
                                   end_node: Optional[str] = None,
                                   title: str = "Dijkstra Rerouting Graph",
                                   output_file: str = "dijkstra_graph.png",
                                   layout: str = "spring"):
    """
    Visualize graph with special highlighting for start and end nodes.
    Shows all elements: blocked edges (red), start node (yellow), end node (orange),
    normal edges (black), and Dijkstra path (green).
    
    Args:
        G: NetworkX directed graph
        edge_attrs: Dictionary of edge attributes
        blocked_edges: Set of blocked edge IDs to highlight
        highlight_path: List of edge IDs representing a path to highlight
        start_node: Start node to highlight (yellow)
        end_node: End node to highlight (orange)
        title: Title for the plot
        output_file: Output filename
        layout: Layout algorithm
    """
    if len(G.nodes()) == 0:
        print("[WARNING] Graph is empty - cannot visualize")
        return
    
    if blocked_edges is None:
        blocked_edges = set()
    
    if possible_routes is None:
        possible_routes = []
    
    # Choose layout - increase spacing significantly to avoid overlaps
    num_nodes = len(G.nodes())
    # Increase k_value (spacing) to prevent node overlaps
    k_value = 4.0 if num_nodes > 50 else 3.5 if num_nodes > 20 else 2.5
    iterations = 200 if num_nodes > 100 else 150
    
    if layout == "spring":
        pos = nx.spring_layout(G, k=k_value, iterations=iterations, seed=42)
        # Adjust positions to ensure nodes don't overlap
        # Increase minimum distance between nodes
        for i, node1 in enumerate(G.nodes()):
            for j, node2 in enumerate(G.nodes()):
                if i < j:
                    x1, y1 = pos[node1]
                    x2, y2 = pos[node2]
                    dist = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
                    min_dist = 0.3  # Minimum distance between nodes
                    if dist < min_dist and dist > 0:
                        # Push nodes apart
                        dx = (x2 - x1) / dist * min_dist
                        dy = (y2 - y1) / dist * min_dist
                        pos[node2] = (x1 + dx, y1 + dy)
    else:
        pos = nx.spring_layout(G, k=k_value, iterations=iterations, seed=42)
    
    # Create figure - larger for better visibility
    figsize = (20, 16) if num_nodes > 100 else (18, 14) if num_nodes > 50 else (16, 12)
    fig, ax = plt.subplots(figsize=figsize, facecolor='white')
    
    # Determine node colors and sizes - MAXIMIZE node appearance
    # Priority: Alternative Routes > Dijkstra Path > Blocked > Normal
    # Check which nodes are in possible routes
    nodes_in_possible_routes = set()
    if possible_routes:
        for route in possible_routes:
            nodes_in_possible_routes.update(route)
    
    # Debug: Print what we're looking for
    print(f"  [DEBUG] Node coloring:")
    print(f"    Start node: {start_node}")
    print(f"    End node: {end_node}")
    print(f"    Highlight path: {highlight_path}")
    print(f"    Path nodes in graph: {[n for n in (highlight_path or []) if n in G.nodes()]}")
    print(f"    Blocked edges: {[n for n in (blocked_edges or []) if n in G.nodes()]}")
    
    node_colors = []
    node_sizes = []
    for node in G.nodes():
        if start_node and node == start_node:
            node_colors.append('#ffff00')  # Yellow for start
            node_sizes.append(1200)  # Very large for start
            print(f"    Node {node}: YELLOW (start)")
        elif end_node and node == end_node:
            node_colors.append('#ff8800')  # Orange for end
            node_sizes.append(1200)  # Very large for end
            print(f"    Node {node}: ORANGE (end)")
        elif node in nodes_in_possible_routes and (not highlight_path or node not in highlight_path):
            # Nodes in alternative routes - show FIRST with distinct colors
            # Use colors matching the route edges - make them more visible
            node_colors.append('#4169E1')  # Royal Blue for alternative route nodes
            node_sizes.append(900)  # Large for alternative route nodes (same size as path nodes)
        elif highlight_path and node in highlight_path:
            # Dijkstra's chosen path nodes - shown SECOND
            node_colors.append('#00ff00')  # Bright green for chosen path nodes
            node_sizes.append(900)  # Large for chosen path nodes
            print(f"    Node {node}: GREEN (path)")
        elif blocked_edges and node in blocked_edges:
            # Blocked nodes - shown THIRD
            node_colors.append('#ff0000')  # Red for blocked nodes
            node_sizes.append(800)  # Large for blocked
            print(f"    Node {node}: RED (blocked)")
        else:
            node_colors.append('#1f77b4')  # Blue for normal (matching image)
            node_sizes.append(600)  # Large for normal nodes
    
    # Determine edge colors and widths - show all edge types clearly
    edge_colors = []
    edge_widths = []
    
    # In edge-to-edge graph, nodes ARE edges, so blocked_edges contains node IDs
    # An edge (connection) is blocked if either endpoint node is in blocked_edges
    for edge in G.edges():
        from_node, to_node = edge
        
        # Check if this edge connection is blocked
        # In edge-to-edge graph: if either the from_node or to_node is a blocked edge
        is_blocked = (from_node in blocked_edges or to_node in blocked_edges)
        
        # Also check edge attributes for blocked status
        edge_key = edge
        edge_attr = edge_attrs.get(edge_key, {})
        if edge_attr.get('is_blocked', False):
            is_blocked = True
        
        # Also check if the edge_id itself is in blocked_edges
        edge_id = edge_attr.get('edge_id', None)
        if edge_id and edge_id in blocked_edges:
            is_blocked = True
        
        # Check if this edge is part of the Dijkstra path (chosen path)
        is_path_edge = False
        if highlight_path and len(highlight_path) > 0:
            # Check if this edge connects consecutive nodes in the path
            for i in range(len(highlight_path) - 1):
                if highlight_path[i] == from_node and highlight_path[i + 1] == to_node:
                    is_path_edge = True
                    break
            # Also check if the edge itself is in the path (for edge-to-edge graphs)
            if not is_path_edge and from_node in highlight_path:
                idx = highlight_path.index(from_node)
                # If this is the last edge in path, check if to_node is the destination
                if idx == len(highlight_path) - 1:
                    # This might be the final connection
                    pass
                elif idx < len(highlight_path) - 1 and highlight_path[idx + 1] == to_node:
                    is_path_edge = True
        
        # Check if this edge is part of any possible alternative route
        is_possible_route_edge = False
        route_index = -1
        if possible_routes and not is_path_edge and not is_blocked:
            for route_idx, route in enumerate(possible_routes):
                if from_node in route and to_node in route:
                    route_idx_in_path = route.index(from_node) if from_node in route else -1
                    if route_idx_in_path >= 0 and route_idx_in_path < len(route) - 1 and route[route_idx_in_path + 1] == to_node:
                        is_possible_route_edge = True
                        route_index = route_idx
                        break
                # Also check if this edge connects consecutive nodes in the route
                elif len(route) > 1:
                    for i in range(len(route) - 1):
                        if route[i] == from_node and route[i + 1] == to_node:
                            is_possible_route_edge = True
                            route_index = route_idx
                            break
                    if is_possible_route_edge:
                        break
        
        # Priority: Possible Routes > Chosen Path (Dijkstra) > Blocked > Normal
        # Show alternative routes first, then Dijkstra path, then blocked edges
        if is_possible_route_edge:
            # Use different colors for different alternative routes - show these FIRST
            alt_colors = ['#4169E1', '#32CD32', '#FFD700', '#FF6347', '#9370DB']  # Royal Blue, Lime Green, Gold, Tomato, Medium Purple
            color_idx = route_index % len(alt_colors)
            edge_colors.append(alt_colors[color_idx])  # Different color for each alternative route
            edge_widths.append(3.5)  # Thick for alternative routes to make them visible
        elif is_path_edge:
            # Dijkstra's chosen path - shown SECOND
            edge_colors.append('#00ff00')  # Bright green for chosen Dijkstra path
            edge_widths.append(4.5)  # Very thick for chosen path (thicker than alternatives)
        elif is_blocked:
            # Blocked edges - shown THIRD
            edge_colors.append('#ff0000')  # Red for blocked edges
            edge_widths.append(3.0)  # Thick for blocked
        else:
            edge_colors.append('#000000')  # Black for normal edges (matching image)
            edge_widths.append(1.5)  # Medium thickness for visibility
    
    # Draw edges in layers for proper visual hierarchy
    # Create lists of edges by type
    all_edges_list = list(G.edges())
    
    # Layer 1: Normal edges (drawn first, in background)
    normal_edges = [e for e, c in zip(all_edges_list, edge_colors) if c == '#000000']
    if normal_edges:
        normal_colors_list = [edge_colors[all_edges_list.index(e)] for e in normal_edges]
        normal_widths_list = [edge_widths[all_edges_list.index(e)] for e in normal_edges]
        nx.draw_networkx_edges(G, pos, ax=ax, edgelist=normal_edges,
                               edge_color=normal_colors_list, width=normal_widths_list,
                               alpha=0.6, arrows=True, arrowsize=15, arrowstyle='->',
                               connectionstyle='arc3,rad=0.1')
    
    # Layer 2: Blocked edges (drawn second)
    blocked_edges_list = [e for e, c in zip(all_edges_list, edge_colors) if c == '#ff0000']
    if blocked_edges_list:
        blocked_colors_list = [edge_colors[all_edges_list.index(e)] for e in blocked_edges_list]
        blocked_widths_list = [edge_widths[all_edges_list.index(e)] for e in blocked_edges_list]
        nx.draw_networkx_edges(G, pos, ax=ax, edgelist=blocked_edges_list,
                               edge_color=blocked_colors_list, width=blocked_widths_list,
                               alpha=0.9, arrows=True, arrowsize=20, arrowstyle='->',
                               connectionstyle='arc3,rad=0.1')
    
    # Layer 3: Dijkstra chosen path (drawn third)
    path_edges_list = [e for e, c in zip(all_edges_list, edge_colors) if c == '#00ff00']
    if path_edges_list:
        path_colors_list = [edge_colors[all_edges_list.index(e)] for e in path_edges_list]
        path_widths_list = [edge_widths[all_edges_list.index(e)] for e in path_edges_list]
        nx.draw_networkx_edges(G, pos, ax=ax, edgelist=path_edges_list,
                               edge_color=path_colors_list, width=path_widths_list,
                               alpha=1.0, arrows=True, arrowsize=25, arrowstyle='->',
                               connectionstyle='arc3,rad=0.15')
    
    # Layer 4: Alternative routes (drawn last, on top) - these are NOT black, red, or green
    alt_route_edges = [e for e, c in zip(all_edges_list, edge_colors) 
                      if c not in ['#000000', '#ff0000', '#00ff00']]
    if alt_route_edges:
        alt_colors_list = [edge_colors[all_edges_list.index(e)] for e in alt_route_edges]
        alt_widths_list = [edge_widths[all_edges_list.index(e)] for e in alt_route_edges]
        print(f"  [DEBUG] Drawing {len(alt_route_edges)} alternative route edges with colors: {set(alt_colors_list)}")
        nx.draw_networkx_edges(G, pos, ax=ax, edgelist=alt_route_edges,
                               edge_color=alt_colors_list, width=alt_widths_list,
                               alpha=0.95, arrows=True, arrowsize=22, arrowstyle='->',
                               connectionstyle='arc3,rad=0.12')
    else:
        print(f"  [DEBUG] No alternative route edges found to draw")
    
    # Draw nodes with MAXIMUM visibility
    nx.draw_networkx_nodes(G, pos, ax=ax,
                           node_color=node_colors,
                           node_size=node_sizes,
                           alpha=0.95,  # Very opaque
                           edgecolors='#2c3e50',  # Dark blue outline (matching image style)
                           linewidths=2.5)  # Thick outline
    
    # Draw labels for ALL nodes with offset to avoid overlap with nodes
    labels = {node: node for node in G.nodes()}
    
    # Calculate label positions with offset from nodes to prevent overlap
    label_positions = {}
    for node in G.nodes():
        x, y = pos[node]
        # Offset labels above nodes to avoid overlap
        # Adjust offset based on node size
        node_size_factor = node_sizes[list(G.nodes()).index(node)] / 1000.0
        offset_y = 0.15 + node_size_factor * 0.1  # Offset above node
        label_positions[node] = (x, y + offset_y)
    
    # Draw all labels with offset positions
    for node, label_text in labels.items():
        x, y = label_positions[node]
        # Truncate long labels
        display_text = label_text[:20] + '...' if len(label_text) > 20 else label_text
        
        # Special styling for important nodes
        if node == start_node:
            bbox_style = dict(boxstyle='round,pad=0.4', facecolor='yellow', alpha=0.9, 
                            edgecolor='darkorange', linewidth=2)
        elif node == end_node:
            bbox_style = dict(boxstyle='round,pad=0.4', facecolor='orange', alpha=0.9, 
                            edgecolor='darkorange', linewidth=2)
        elif node in blocked_edges:
            bbox_style = dict(boxstyle='round,pad=0.4', facecolor='lightcoral', alpha=0.9, 
                            edgecolor='red', linewidth=2)
        elif highlight_path and node in highlight_path:
            bbox_style = dict(boxstyle='round,pad=0.4', facecolor='lightgreen', alpha=0.9, 
                            edgecolor='darkgreen', linewidth=2)
        else:
            bbox_style = dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9, 
                            edgecolor='gray', linewidth=1)
        
        ax.text(x, y, display_text, 
               fontsize=9, fontweight='normal', color='black',
               ha='center', va='bottom',
               bbox=bbox_style)
    
    # Also label blocked edges more prominently
    if blocked_edges:
        blocked_nodes_in_graph = [n for n in G.nodes() if n in blocked_edges]
        for blocked_node in blocked_nodes_in_graph:
            if blocked_node in pos:
                x, y = pos[blocked_node]
                # Add a "BLOCKED" annotation below the node
                ax.annotate('BLOCKED', xy=(x, y), xytext=(x, y - 0.2),
                           fontsize=8, fontweight='bold', color='red',
                           ha='center', va='top',
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='red', 
                                   alpha=0.7, edgecolor='darkred', linewidth=2),
                           arrowprops=dict(arrowstyle='->', color='red', lw=1.5))
    
    # Add comprehensive legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#1f77b4', edgecolor='#2c3e50', linewidth=2, label='Normal Edge (Node)'),
        Patch(facecolor='#ff0000', edgecolor='#2c3e50', linewidth=2, label='Blocked Edge (Train Crossing)'),
    ]
    
    if start_node:
        legend_elements.append(Patch(facecolor='#ffff00', edgecolor='#2c3e50', linewidth=2, label='Start Node'))
    if end_node:
        legend_elements.append(Patch(facecolor='#ff8800', edgecolor='#2c3e50', linewidth=2, label='End Node'))
    if highlight_path:
        legend_elements.append(Patch(facecolor='#00ff00', edgecolor='#2c3e50', linewidth=2, label='Chosen Path (Dijkstra)'))
    if possible_routes:
        legend_elements.append(Patch(facecolor='#90EE90', edgecolor='#2c3e50', linewidth=2, label='Alternative Route Nodes'))
    
    # Also add edge type legend - ORDER: Routes > Dijkstra > Blocked > Normal
    from matplotlib.lines import Line2D
    # Show alternative routes FIRST in legend
    if possible_routes:
        # Add legend for alternative routes - shown FIRST
        alt_colors = ['#4169E1', '#32CD32', '#FFD700', '#FF6347', '#9370DB']
        for i, route in enumerate(possible_routes[:5]):  # Show up to 5 alternative routes
            color = alt_colors[i % len(alt_colors)]
            legend_elements.append(Line2D([0], [0], color=color, lw=3.5, 
                                        label=f'Alternative Route {i+1} ({len(route)} edges)'))
    # Show Dijkstra path SECOND
    if highlight_path:
        legend_elements.append(Line2D([0], [0], color='#00ff00', lw=4.5, label='Dijkstra Chosen Path'))
    # Show blocked edges THIRD
    if blocked_edges:
        legend_elements.append(Line2D([0], [0], color='#ff0000', lw=3, label='Blocked Edge (Connection)'))
    # Show normal edges LAST
    legend_elements.append(Line2D([0], [0], color='#000000', lw=2.5, label='Normal Edge (Connection)'))
    
    ax.legend(handles=legend_elements, loc='upper right', fontsize=11, framealpha=0.9)
    
    # Set title
    ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
    
    # Add stats box (matching image style)
    stats_text = f'Nodes: {len(G.nodes())}\nEdges: {len(G.edges())}'
    if blocked_edges:
        blocked_count = len([n for n in G.nodes() if n in blocked_edges])
        if blocked_count > 0:
            stats_text += f'\nBlocked: {blocked_count}'
    if highlight_path:
        stats_text += f'\nPath Length: {len(highlight_path)} edges'
    
    ax.text(0.02, 0.98, 
           stats_text,
           transform=ax.transAxes,
           fontsize=11,
           verticalalignment='top',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7, edgecolor='black', linewidth=1))
    
    # Add REROUTING SCENARIO SUMMARY box - show what's being visualized
    scenario_text = "REROUTING SCENARIO:\n"
    scenario_text += "=" * 30 + "\n"
    
    if possible_routes:
        scenario_text += f"✓ {len(possible_routes)} ALTERNATIVE ROUTES\n"
        scenario_text += "  (Colored lines: Royal Blue, Lime, Gold, etc.)\n"
    else:
        scenario_text += "✗ No alternative routes found\n"
    
    if highlight_path:
        scenario_text += f"✓ DIJKSTRA CHOSEN PATH\n"
        scenario_text += f"  (Bright Green, {len(highlight_path)} edges)\n"
    else:
        scenario_text += "✗ No Dijkstra path\n"
    
    if blocked_edges:
        blocked_count = len([n for n in G.nodes() if n in blocked_edges])
        scenario_text += f"✓ {blocked_count} BLOCKED EDGES\n"
        scenario_text += "  (Red nodes & connections)\n"
    else:
        scenario_text += "✗ No blocked edges\n"
    
    ax.text(0.02, 0.25, 
           scenario_text,
           transform=ax.transAxes,
           fontsize=10,
           verticalalignment='top',
           bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.95, 
                   edgecolor='darkorange', linewidth=2.5))
    
    # Add blocked edges list box to show all blocked edges clearly
    if blocked_edges:
        blocked_nodes_in_graph = [n for n in G.nodes() if n in blocked_edges]
        if blocked_nodes_in_graph:
            blocked_text = "BLOCKED EDGES:\n"
            # Show first 5 blocked edges
            for i, blocked_node in enumerate(blocked_nodes_in_graph[:5]):
                blocked_text += f"{i+1}. {blocked_node[:25]}\n"
            if len(blocked_nodes_in_graph) > 5:
                blocked_text += f"... and {len(blocked_nodes_in_graph) - 5} more"
            
            ax.text(0.02, 0.10, 
                   blocked_text,
                   transform=ax.transAxes,
                   fontsize=9,
                   verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.9, 
                           edgecolor='red', linewidth=2))
    
    # Add explanation text box to help understand the visualization
    explanation_text = "HOW TO READ THIS GRAPH:\n"
    explanation_text += "• Nodes = Road edges in SUMO network\n"
    explanation_text += "• Arrows = Direction vehicles can travel\n"
    explanation_text += "\nVISUAL PRIORITY ORDER:\n"
    if possible_routes:
        explanation_text += f"1. ALTERNATIVE ROUTES ({len(possible_routes)} routes)\n"
        explanation_text += "   - Colored lines (Royal Blue, Lime, Gold, etc.)\n"
        explanation_text += "   - Royal Blue nodes = Route nodes\n"
    if highlight_path:
        explanation_text += "2. DIJKSTRA CHOSEN PATH\n"
        explanation_text += "   - Bright Green thick lines\n"
        explanation_text += "   - Bright Green nodes = Path nodes\n"
    if blocked_edges:
        explanation_text += "3. BLOCKED EDGES\n"
        explanation_text += "   - Red lines and nodes\n"
        explanation_text += "   - 'BLOCKED' labels below nodes\n"
    explanation_text += "4. NORMAL EDGES\n"
    explanation_text += "   - Blue nodes, Black lines\n"
    if start_node:
        explanation_text += f"\n• Yellow = START (vehicle's current edge)\n"
    if end_node:
        explanation_text += f"• Orange = END (vehicle's destination edge)\n"
    
    ax.text(0.02, 0.02, 
           explanation_text,
           transform=ax.transAxes,
           fontsize=9,
           verticalalignment='bottom',
           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8, edgecolor='darkblue', linewidth=1.5))
    
    # Add path details if available
    if highlight_path and start_node and end_node:
        path_details = f"CHOSEN PATH (Dijkstra):\n"
        path_details += f"From: {start_node[:25]}\n"
        path_details += f"To: {end_node[:25]}\n"
        path_details += f"Length: {len(highlight_path)} edges\n"
        path_details += f"Route: {' → '.join(highlight_path[:2])}"
        if len(highlight_path) > 2:
            path_details += f" → ... → {highlight_path[-1]}"
        
        ax.text(0.98, 0.25, 
               path_details,
               transform=ax.transAxes,
               fontsize=9,
               verticalalignment='top',
               horizontalalignment='right',
               bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8, edgecolor='darkgreen', linewidth=1.5))
    
    # Add alternative routes details if available
    if possible_routes:
        alt_routes_text = f"ALTERNATIVE ROUTES ({len(possible_routes)}):\n"
        for i, route in enumerate(possible_routes[:3]):  # Show first 3 alternatives
            alt_routes_text += f"Route {i+1}: {len(route)} edges\n"
            if len(route) <= 4:
                alt_routes_text += f"  {' → '.join(route)}\n"
            else:
                alt_routes_text += f"  {route[0]} → ... → {route[-1]}\n"
        if len(possible_routes) > 3:
            alt_routes_text += f"... and {len(possible_routes) - 3} more"
        
        ax.text(0.98, 0.02, 
               alt_routes_text,
               transform=ax.transAxes,
               fontsize=8,
               verticalalignment='bottom',
               horizontalalignment='right',
               bbox=dict(boxstyle='round', facecolor='lavender', alpha=0.8, edgecolor='purple', linewidth=1.5))
    
    ax.axis('off')
    plt.tight_layout()
    
    # Save figure with high quality
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()


def main():
    """Main function to run graph visualization."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Visualize Dijkstra rerouting graph')
    parser.add_argument('--config', type=str, default='4thave.sumocfg',
                       help='SUMO config file path')
    parser.add_argument('--output', type=str, default='dijkstra_graph.png',
                       help='Output image filename')
    parser.add_argument('--layout', type=str, default='spring',
                       choices=['spring', 'circular', 'kamada_kawai', 'planar'],
                       help='Graph layout algorithm')
    parser.add_argument('--path', type=str, nargs='+', default=None,
                       help='Path to highlight (space-separated edge IDs)')
    parser.add_argument('--all-edges', action='store_true', dest='all_edges',
                       help='Show all edges from network, not just those with connections')
    
    args = parser.parse_args()
    
    # Check if SUMO config exists
    if not os.path.exists(args.config):
        print(f"[ERROR] SUMO config file not found: {args.config}")
        return
    
    # Highlight path if provided
    highlight_path = args.path if args.path else None
    
    # Visualize
    visualize_from_running_simulation(args.config, args.output, highlight_path, args.layout, args.all_edges)


if __name__ == "__main__":
    main()

