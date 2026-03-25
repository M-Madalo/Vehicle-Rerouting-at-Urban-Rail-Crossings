"""
No-Train Analysis Script
Runs SUMO simulation WITHOUT trains present, collects performance metrics including LOS,
and generates visualizations. This provides a baseline for comparison with train scenarios.
"""

import sys
import os
import time
import traci
import json
import xml.etree.ElementTree as ET
from enhanced_dijkstra import EnhancedDijkstra
from sumo_controller import SUMOController
from metrics_collector import MetricsCollector
from visualize_metrics import MetricsVisualizer


def create_no_train_config():
    """Create a SUMO config file that uses routes without trains."""
    
    original_config = "4thave.sumocfg"
    no_train_config = "4thave_no_trains.sumocfg"
    
    # First, ensure we have a routes file without trains
    if not os.path.exists("trips_no_trains.rou.xml"):
        print("[INFO] Creating routes file without trains...")
        from create_no_train_routes import create_no_train_routes
        if not create_no_train_routes():
            print("[ERROR] Failed to create routes file without trains")
            return None
    
    # Read original config
    tree = ET.parse(original_config)
    root = tree.getroot()
    
    # Find and update route file reference
    route_files_updated = False
    for input_elem in root.findall('.//input'):
        route_files = input_elem.find('route-files')
        if route_files is not None:
            old_value = route_files.get('value', '')
            route_files.set('value', 'trips_no_trains.rou.xml')
            route_files_updated = True
            print(f"[OK] Updated route file from '{old_value}' to: trips_no_trains.rou.xml")
    
    if not route_files_updated:
        # Try alternative structure
        for route_files in root.findall('.//route-files'):
            old_value = route_files.get('value', '')
            route_files.set('value', 'trips_no_trains.rou.xml')
            route_files_updated = True
            print(f"[OK] Updated route file from '{old_value}' to: trips_no_trains.rou.xml")
    
    # Save modified config
    tree.write(no_train_config, encoding='UTF-8', xml_declaration=True)
    print(f"[OK] Created config file: {no_train_config}")
    
    return no_train_config


def run_no_train_analysis():
    """Run no-train scenario analysis."""
    
    # Configuration
    SUMO_CONFIG = "4thave.sumocfg"
    SIMULATION_END_TIME = 3600  # 1 hour in seconds
    STEP_LENGTH = 1.0  # 1 second per step
    PRINT_STATS_INTERVAL = 60  # Print statistics every 60 seconds
    
    print("=" * 70)
    print("NO-TRAIN ANALYSIS - Baseline Performance Without Trains")
    print("Performance Metrics Collection with LOS Calculation")
    print("=" * 70)
    print()
    
    # Create config file without trains
    print("Preparing simulation without trains...")
    no_train_config = create_no_train_config()
    if not no_train_config:
        print("[ERROR] Failed to create no-train configuration")
        sys.exit(1)
    
    SUMO_CONFIG = no_train_config
    
    # Check if SUMO config exists
    if not os.path.exists(SUMO_CONFIG):
        print(f"[ERROR] SUMO configuration file '{SUMO_CONFIG}' not found!")
        sys.exit(1)
    
    # Initialize enhanced Dijkstra algorithm (not used, but required)
    print("Initializing Enhanced Dijkstra Algorithm...")
    dijkstra = EnhancedDijkstra(network_graph={})
    print("[OK] Enhanced Dijkstra initialized")
    print()
    
    # Initialize SUMO controller with rerouting DISABLED (baseline)
    print("Initializing SUMO Controller (No Trains, No Rerouting)...")
    controller = SUMOController(SUMO_CONFIG, dijkstra, enable_rerouting=False)
    controller.sumo_binary = "sumo-gui"  # Use GUI version for visualization
    print("[OK] SUMO Controller initialized")
    print("[INFO] Running in NO-TRAIN mode - trains will not appear")
    print("[INFO] Metrics will be collected for baseline comparison")
    print("[INFO] SUMO GUI will open - you can watch the simulation")
    print()
    
    # Initialize metrics collector
    print("Initializing Metrics Collector...")
    metrics_collector = MetricsCollector()
    print("[OK] Metrics Collector initialized")
    print()
    
    try:
        # Start simulation
        print("Starting SUMO simulation...")
        try:
            controller.start_simulation()
            print("[OK] Simulation started")
            # Give GUI time to initialize
            if controller.sumo_binary == "sumo-gui":
                print("[INFO] Waiting for SUMO GUI to initialize...")
                time.sleep(2.0)  # Give GUI time to open and render
                print("[INFO] SUMO GUI should now be visible")
                print("[INFO] If simulation is paused, click Play button in SUMO GUI")
        except Exception as e:
            print(f"[ERROR] Error starting simulation: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
        
        # Initialize edge information for metrics collector
        print("Loading network information...")
        metrics_collector.initialize_edge_info()
        print(f"[OK] Loaded information for {len(metrics_collector.edge_info)} edges")
        print()
        
        # Run simulation
        print("Running simulation...")
        print(f"Simulation will run for {SIMULATION_END_TIME} seconds")
        print(f"Statistics will be printed every {PRINT_STATS_INTERVAL} seconds")
        print()
        print("[INFO] No trains will appear in this simulation")
        print("[INFO] This provides baseline performance for comparison")
        print()
        
        start_time = time.time()
        current_sim_time = 0.0
        last_stats_time = 0.0
        
        while current_sim_time < SIMULATION_END_TIME:
            # Run one simulation step
            controller.run_step(current_sim_time)
            
            # Collect metrics
            # Note: No trains, so no active crossings
            active_crossings_list = []  # No trains = no crossings
            blocked_edges = set()  # No trains = no blocked edges
            
            # Update vehicle metrics
            metrics_collector.update_vehicle_metrics(
                current_sim_time,
                active_crossings_list,
                blocked_edges,
                controller.train_vehicles,  # Will be empty
                controller.vehicle_routes
            )
            
            # Update crossing metrics (will be empty, but still call for consistency)
            vehicles_to_check = [v for v in traci.vehicle.getIDList() 
                               if v not in controller.train_vehicles]
            metrics_collector.update_crossing_metrics(
                current_sim_time,
                active_crossings_list,
                vehicles_to_check,
                controller.vehicle_routes
            )
            
            # Track unique vehicles (important for statistics)
            for veh_id in vehicles_to_check:
                metrics_collector.total_vehicles_seen.add(veh_id)
            
            # Print periodic statistics
            if current_sim_time - last_stats_time >= PRINT_STATS_INTERVAL:
                stats = metrics_collector.get_statistics()
                print(f"\n[{current_sim_time:.1f}s] Performance Statistics:")
                print(f"  Active vehicles: {len(traci.vehicle.getIDList())}")
                print(f"  Active crossings: 0 (no trains)")
                if stats:
                    print(f"  Avg travel time: {stats.get('avg_travel_time_all', 0):.1f}s")
                    print(f"  Avg delay: {stats.get('avg_delay_time_all', 0):.1f}s")
                    print(f"  Avg speed: {stats.get('avg_speed_all', 0)*2.237:.1f} mph")
                    print(f"  LOS: {stats.get('los_all', 'N/A')}")
                last_stats_time = current_sim_time
            
            # Advance simulation
            traci.simulationStep()
            current_sim_time = traci.simulation.getTime()
        
        # Final statistics
        print("\n" + "=" * 70)
        print("SIMULATION COMPLETE")
        print("=" * 70)
        print()
        
        # Calculate final metrics
        print("Calculating final metrics...")
        # Use get_statistics() to include total_vehicles_seen
        final_stats = metrics_collector.get_statistics()
        
        # Save statistics
        output_file = "stats_no_train.json"
        print(f"Saving statistics to {output_file}...")
        with open(output_file, 'w') as f:
            json.dump(final_stats, f, indent=2)
        print(f"[OK] Statistics saved to {output_file}")
        
        # Print summary
        print("\n" + "=" * 70)
        print("NO-TRAIN ANALYSIS SUMMARY")
        print("=" * 70)
        if final_stats:
            print(f"\nOverall Performance (No Trains):")
            print(f"  Total vehicles: {final_stats.get('total_vehicles_seen', 0)}")
            print(f"  Average travel time: {final_stats.get('avg_travel_time_all', 0):.1f} seconds")
            print(f"  Average delay: {final_stats.get('avg_delay_time_all', 0):.1f} seconds")
            print(f"  Average speed: {final_stats.get('avg_speed_all', 0)*2.237:.1f} mph")
            print(f"  Free-flow speed: {final_stats.get('avg_free_flow_speed_all', 0)*2.237:.1f} mph")
            print(f"  Speed ratio: {final_stats.get('speed_ratio_all', 0)*100:.1f}%")
            print(f"  Route efficiency: {final_stats.get('route_efficiency_all', 1.0)*100:.1f}%")
            print(f"  Level of Service (LOS): {final_stats.get('los_all', 'N/A')}")
            print(f"  Vehicles affected by crossings: {final_stats.get('vehicles_affected_by_crossings', 0)} (should be 0)")
        
        # Generate visualizations
        print("\nGenerating visualizations...")
        visualizer = MetricsVisualizer()
        
        # Plot metrics
        visualizer.plot_baseline_metrics(final_stats, "no_train_metrics.png")
        
        # Compare with baseline (with trains) if available
        baseline_file = "stats_baseline.json"
        if os.path.exists(baseline_file):
            print(f"\nComparing with baseline results (with trains) from {baseline_file}...")
            with open(baseline_file, 'r') as f:
                baseline_stats = json.load(f)
            
            visualizer.plot_comparison(baseline_stats, final_stats, "train_vs_no_train_comparison.png")
            print("[OK] Comparison plot generated")
            
            # Print comparison summary
            print("\n" + "=" * 70)
            print("TRAIN vs NO-TRAIN COMPARISON")
            print("=" * 70)
            if baseline_stats and final_stats:
                baseline_metrics = baseline_stats.get('congestion_metrics', baseline_stats)
                no_train_metrics = final_stats.get('congestion_metrics', final_stats)
                
                baseline_travel = baseline_metrics.get('avg_travel_time_all', 0)
                no_train_travel = no_train_metrics.get('avg_travel_time_all', 0)
                travel_impact = ((baseline_travel - no_train_travel) / no_train_travel * 100) if no_train_travel > 0 else 0
                
                baseline_delay = baseline_metrics.get('avg_delay_time_all', 0)
                no_train_delay = no_train_metrics.get('avg_delay_time_all', 0)
                delay_impact = ((baseline_delay - no_train_delay) / no_train_delay * 100) if no_train_delay > 0 else 0
                
                baseline_speed = baseline_metrics.get('avg_speed_all', 0)
                no_train_speed = no_train_metrics.get('avg_speed_all', 0)
                speed_impact = ((no_train_speed - baseline_speed) / baseline_speed * 100) if baseline_speed > 0 else 0
                
                print(f"\nTravel Time:")
                print(f"  No Train: {no_train_travel:.1f}s")
                print(f"  With Train: {baseline_travel:.1f}s")
                print(f"  Train Impact: {travel_impact:+.1f}%")
                
                print(f"\nDelay:")
                print(f"  No Train: {no_train_delay:.1f}s")
                print(f"  With Train: {baseline_delay:.1f}s")
                print(f"  Train Impact: {delay_impact:+.1f}%")
                
                print(f"\nSpeed:")
                print(f"  No Train: {no_train_speed*2.237:.1f} mph")
                print(f"  With Train: {baseline_speed*2.237:.1f} mph")
                print(f"  Train Impact: {speed_impact:+.1f}%")
                
                print(f"\nLevel of Service:")
                print(f"  No Train: LOS {final_stats.get('los_all', 'N/A')}")
                print(f"  With Train: LOS {baseline_stats.get('los_all', 'N/A')}")
        
        elapsed_time = time.time() - start_time
        print(f"\n[OK] Analysis complete in {elapsed_time:.1f} seconds")
        print(f"[OK] Visualizations saved to metrics_plots/ directory")
        print()
        print("[INFO] Use this data to compare:")
        print("  - Baseline (with trains, no rerouting)")
        print("  - No-train baseline (this run)")
        print("  - Rerouting (with trains, with rerouting)")
        
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
        traci.close()
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Error during simulation: {e}")
        import traceback
        traceback.print_exc()
        traci.close()
        sys.exit(1)
    finally:
        traci.close()


if __name__ == "__main__":
    run_no_train_analysis()

