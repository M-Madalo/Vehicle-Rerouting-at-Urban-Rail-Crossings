"""
Main Simulation Runner for Dynamic Vehicle Rerouting Research Project
Enhanced AI Dijkstra Algorithm for Train Crossing Scenarios

This script runs the SUMO simulation with dynamic vehicle rerouting
when trains are crossing.
"""

import sys
import os
import time
import traci
import json
from enhanced_dijkstra import EnhancedDijkstra
from sumo_controller import SUMOController
from visualize_metrics import MetricsVisualizer


def main():
    """Main function to run the simulation."""
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Dynamic Vehicle Rerouting Simulation')
    parser.add_argument('--baseline', action='store_true', 
                       help='Run in baseline mode (no rerouting) to collect baseline metrics')
    parser.add_argument('--no-gui', action='store_true',
                       help='Run without GUI (faster)')
    args = parser.parse_args()
    
    # Configuration
    SUMO_CONFIG = "4thave.sumocfg"
    SIMULATION_END_TIME = 3600  # 1 hour in seconds
    STEP_LENGTH = 1.0  # 1 second per step
    PRINT_STATS_INTERVAL = 60  # Print statistics every 60 seconds
    ENABLE_REROUTING = not args.baseline  # Disable rerouting in baseline mode
    
    print("=" * 70)
    print("Dynamic Vehicle Rerouting Research Project")
    print("Enhanced AI Dijkstra Algorithm for Train Crossings")
    if args.baseline:
        print("*** BASELINE MODE: Rerouting DISABLED - Collecting baseline metrics ***")
    else:
        print("*** REROUTING MODE: Rerouting ENABLED ***")
    print("=" * 70)
    print()
    
    # Check if SUMO config exists
    if not os.path.exists(SUMO_CONFIG):
        print(f"Error: SUMO configuration file '{SUMO_CONFIG}' not found!")
        print("Please ensure the file exists in the current directory.")
        sys.exit(1)
    
    # Initialize enhanced Dijkstra algorithm
    print("Initializing Enhanced Dijkstra Algorithm...")
    dijkstra = EnhancedDijkstra(network_graph={})
    print("[OK] Enhanced Dijkstra initialized")
    print()
    
    # Initialize SUMO controller
    print("Initializing SUMO Controller...")
    controller = SUMOController(SUMO_CONFIG, dijkstra, enable_rerouting=ENABLE_REROUTING)
    if args.no_gui:
        controller.sumo_binary = "sumo"  # Use non-GUI version
    print("[OK] SUMO Controller initialized")
    if args.baseline:
        print("[INFO] Running in BASELINE mode - no rerouting will occur")
        print("[INFO] Metrics will still be collected for comparison")
    print()
    
    try:
        # Start simulation
        print("Starting SUMO simulation...")
        try:
            controller.start_simulation()
            print("[OK] Simulation started")
        except Exception as e:
            print(f"❌ Error starting simulation: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
        
        # If using GUI, give it a moment to fully initialize
        if controller.sumo_binary == "sumo-gui":
            print("   Waiting for GUI to initialize...")
            time.sleep(1.0)  # Give GUI time to open and render
            print("   GUI should now be visible")
            print("   [WARNING] If the simulation is paused, click the Play button in SUMO GUI")
        print()
        
        # Run simulation
        print("Running simulation...")
        print(f"Simulation will run for {SIMULATION_END_TIME} seconds")
        print(f"Statistics will be printed every {PRINT_STATS_INTERVAL} seconds")
        print()
        print("Note: Trains are scheduled to depart at:")
        print("  - train_1: 300 seconds")
        print("  - train_2: 480 seconds")
        print()
        print("Note: Vehicles have various depart times (starting from ~1.59 seconds)")
        print("      They will appear gradually as the simulation progresses.")
        print()
        print("-" * 70)
        
        # Check initial state (after first step)
        try:
            controller.run_step(0.0)  # Run first step to load vehicles
            print(f"[Time: 0.0s] Initial check:")
            initial_vehicles = len(traci.vehicle.getIDList())
            print(f"  - Vehicles in simulation: {initial_vehicles}")
            if initial_vehicles == 0:
                print("  - [WARNING] No vehicles yet - they will spawn at their depart times")
                print("  - First vehicle departs at ~1.59 seconds")
                print("  - If using GUI, check if simulation is paused (click Play button)")
            else:
                print(f"  - [OK] Vehicles are loading correctly")
            print("-" * 70)
        except Exception as e:
            print(f"[WARNING] Warning during initial step: {e}")
            print("  Continuing simulation...")
            print("-" * 70)
        
        start_time = time.time()
        current_sim_time = 0.0
        last_stats_time = 0.0
        
        while current_sim_time < SIMULATION_END_TIME:
            # Run one simulation step
            controller.run_step(current_sim_time)
            
            # Update simulation time
            current_sim_time += STEP_LENGTH
            
            # Print statistics periodically
            if current_sim_time - last_stats_time >= PRINT_STATS_INTERVAL:
                stats = controller.get_statistics()
                # Also get actual vehicle count from SUMO
                actual_vehicles = len(traci.vehicle.getIDList())
                mode_label = "[BASELINE]" if args.baseline else "[REROUTING]"
                print(f"\n[Time: {current_sim_time:.1f}s] Statistics {mode_label}:")
                print(f"  - Vehicles in SUMO: {actual_vehicles}")
                print(f"  - Current Vehicles (non-train): {stats['total_vehicles']}")
                print(f"  - Total Unique Vehicles Seen: {stats['total_vehicles_seen']}")
                if not args.baseline:
                    print(f"  - Unique Vehicles Rerouted: {stats['unique_vehicles_rerouted']}")
                    print(f"  - Total Reroute Operations: {stats['reroute_count']}")
                    if stats['total_vehicles_seen'] > 0:
                        reroute_percentage = (stats['unique_vehicles_rerouted'] / stats['total_vehicles_seen']) * 100
                        print(f"  - Reroute Rate: {reroute_percentage:.2f}%")
                else:
                    print(f"  - [BASELINE] Rerouting disabled - vehicles follow original routes")
                print(f"  - Active Train Crossings: {stats['active_crossings']}")
                print(f"  - Train Vehicles: {stats['train_vehicles']}")
                if actual_vehicles == 0:
                    print(f"  - [WARNING] No vehicles in simulation yet - check depart times in routes file")
                
                # Congestion and Travel Time Metrics
                if stats.get('avg_travel_time_all', 0) > 0:
                    print(f"\n  CONGESTION & TRAVEL TIME METRICS:")
                    print(f"  - Avg Travel Time (all vehicles): {stats.get('avg_travel_time_all', 0):.2f}s")
                    print(f"  - Avg Delay Time (all vehicles): {stats.get('avg_delay_time_all', 0):.2f}s")
                    print(f"  - Avg Speed (all vehicles): {stats.get('avg_speed_all', 0):.2f} m/s ({stats.get('avg_speed_all', 0)*2.237:.2f} mph)")
                    print(f"  - Vehicles Affected by Crossings: {stats.get('vehicles_affected_by_crossings', 0)}")
                    if stats.get('vehicles_affected_by_crossings', 0) > 0:
                        print(f"  - Avg Crossing Delay: {stats.get('avg_crossing_delay', 0):.2f}s")
                    
                    # Rerouted vs Non-rerouted comparison
                    if stats.get('avg_travel_time_rerouted', 0) > 0 and stats.get('avg_travel_time_non_rerouted', 0) > 0:
                        print(f"\n  REROUTED vs NON-REROUTED COMPARISON:")
                        print(f"  - Avg Travel Time (rerouted): {stats.get('avg_travel_time_rerouted', 0):.2f}s")
                        print(f"  - Avg Travel Time (non-rerouted): {stats.get('avg_travel_time_non_rerouted', 0):.2f}s")
                        travel_time_improvement = stats.get('avg_travel_time_non_rerouted', 0) - stats.get('avg_travel_time_rerouted', 0)
                        if travel_time_improvement > 0:
                            print(f"  - Travel Time Improvement: {travel_time_improvement:.2f}s ({travel_time_improvement/stats.get('avg_travel_time_non_rerouted', 1)*100:.1f}% faster)")
                        print(f"  - Avg Delay (rerouted): {stats.get('avg_delay_rerouted', 0):.2f}s")
                        print(f"  - Avg Delay (non-rerouted): {stats.get('avg_delay_non_rerouted', 0):.2f}s")
                        print(f"  - Avg Speed (rerouted): {stats.get('avg_speed_rerouted', 0)*2.237:.2f} mph")
                        print(f"  - Avg Speed (non-rerouted): {stats.get('avg_speed_non_rerouted', 0)*2.237:.2f} mph")
                        print(f"  - Route Efficiency (rerouted): {stats.get('route_efficiency_rerouted', 1.0):.3f}")
                        print(f"  - Route Efficiency (non-rerouted): {stats.get('route_efficiency_non_rerouted', 1.0):.3f}")
                    
                    # Crossing-specific stats
                    if stats.get('crossing_stats'):
                        print(f"\n  CROSSING-SPECIFIC METRICS:")
                        for crossing_id, crossing_data in stats['crossing_stats'].items():
                            print(f"  - {crossing_id}:")
                            print(f"    * Vehicles Affected: {crossing_data.get('vehicles_affected', 0)}")
                            print(f"    * Max Queue Length: {crossing_data.get('max_queue_length', 0)}")
                            print(f"    * Total Delay: {crossing_data.get('total_delay', 0):.2f}s")
                            print(f"    * Avg Delay per Vehicle: {crossing_data.get('avg_delay_per_vehicle', 0):.2f}s")
                            print(f"    * Crossing Duration: {crossing_data.get('duration', 0):.2f}s")
                    
                    # Edge congestion
                    if stats.get('avg_edge_congestion', 0) > 0:
                        print(f"  - Avg Edge Congestion (vehicles/edge): {stats.get('avg_edge_congestion', 0):.2f}")
                
                print("-" * 70)
                last_stats_time = current_sim_time
        
        # Final statistics
        elapsed_time = time.time() - start_time
        final_stats = controller.get_statistics()
        
        print()
        print("=" * 70)
        print("SIMULATION COMPLETED")
        print("=" * 70)
        print(f"Total Simulation Time: {SIMULATION_END_TIME} seconds")
        print(f"Real Execution Time: {elapsed_time:.2f} seconds")
        print()
        mode_label = "BASELINE" if args.baseline else "REROUTING"
        print(f"Final Statistics ({mode_label} MODE):")
        print(f"  - Total Unique Vehicles Processed: {final_stats['total_vehicles_seen']}")
        if not args.baseline:
            print(f"  - Unique Vehicles Rerouted: {final_stats['unique_vehicles_rerouted']}")
            print(f"  - Total Reroute Operations: {final_stats['reroute_count']}")
            if final_stats['total_vehicles_seen'] > 0:
                reroute_percentage = (final_stats['unique_vehicles_rerouted'] / final_stats['total_vehicles_seen']) * 100
                print(f"  - Overall Reroute Rate: {reroute_percentage:.2f}%")
        else:
            print(f"  - [BASELINE] No rerouting performed")
        print(f"  - Active Train Crossings: {final_stats['active_crossings']}")
        print(f"  - Train Vehicles: {final_stats['train_vehicles']}")
        
        # Final Congestion and Travel Time Metrics
        if final_stats.get('avg_travel_time_all', 0) > 0:
            print(f"\n  FINAL CONGESTION & TRAVEL TIME METRICS:")
            print(f"  - Average Travel Time (all vehicles): {final_stats.get('avg_travel_time_all', 0):.2f}s")
            print(f"  - Average Delay Time (all vehicles): {final_stats.get('avg_delay_time_all', 0):.2f}s")
            print(f"  - Average Speed (all vehicles): {final_stats.get('avg_speed_all', 0)*2.237:.2f} mph")
            print(f"  - Total Vehicles Affected by Crossings: {final_stats.get('vehicles_affected_by_crossings', 0)}")
            if final_stats.get('vehicles_affected_by_crossings', 0) > 0:
                print(f"  - Average Crossing Delay: {final_stats.get('avg_crossing_delay', 0):.2f}s")
            
            # Rerouted vs Non-rerouted comparison
            if final_stats.get('avg_travel_time_rerouted', 0) > 0 and final_stats.get('avg_travel_time_non_rerouted', 0) > 0:
                print(f"\n  REROUTING EFFECTIVENESS:")
                travel_time_improvement = final_stats.get('avg_travel_time_non_rerouted', 0) - final_stats.get('avg_travel_time_rerouted', 0)
                delay_reduction = final_stats.get('avg_delay_non_rerouted', 0) - final_stats.get('avg_delay_rerouted', 0)
                speed_improvement = final_stats.get('avg_speed_rerouted', 0) - final_stats.get('avg_speed_non_rerouted', 0)
                
                print(f"  - Travel Time Improvement: {travel_time_improvement:.2f}s ({travel_time_improvement/final_stats.get('avg_travel_time_non_rerouted', 1)*100:.1f}% faster)")
                print(f"  - Delay Reduction: {delay_reduction:.2f}s ({delay_reduction/final_stats.get('avg_delay_non_rerouted', 1)*100:.1f}% less delay)" if final_stats.get('avg_delay_non_rerouted', 0) > 0 else f"  - Delay Reduction: {delay_reduction:.2f}s")
                print(f"  - Speed Improvement: {speed_improvement*2.237:.2f} mph ({speed_improvement/final_stats.get('avg_speed_non_rerouted', 1)*100:.1f}% faster)" if final_stats.get('avg_speed_non_rerouted', 0) > 0 else f"  - Speed Improvement: {speed_improvement*2.237:.2f} mph")
                print(f"  - Route Efficiency: {final_stats.get('route_efficiency_rerouted', 1.0):.3f} (rerouted) vs {final_stats.get('route_efficiency_non_rerouted', 1.0):.3f} (non-rerouted)")
            
            # Crossing-specific summary
            if final_stats.get('crossing_stats'):
                print(f"\n  CROSSING SUMMARY:")
                total_crossing_delay = sum(c.get('total_delay', 0) for c in final_stats['crossing_stats'].values())
                total_affected = sum(c.get('vehicles_affected', 0) for c in final_stats['crossing_stats'].values())
                max_queue = max((c.get('max_queue_length', 0) for c in final_stats['crossing_stats'].values()), default=0)
                print(f"  - Total Crossings: {len(final_stats['crossing_stats'])}")
                print(f"  - Total Vehicles Affected: {total_affected}")
                print(f"  - Total Crossing Delay: {total_crossing_delay:.2f}s")
                print(f"  - Maximum Queue Length: {max_queue}")
        
        print("=" * 70)
        
        # Save statistics and generate visualizations
        try:
            # Prepare statistics for saving
            stats_to_save = {
                'mode': 'baseline' if args.baseline else 'rerouting',
                'simulation_time': SIMULATION_END_TIME,
                'real_time': elapsed_time,
                'statistics': final_stats,
                'congestion_metrics': {
                    'avg_travel_time_all': final_stats.get('avg_travel_time_all', 0),
                    'avg_delay_time_all': final_stats.get('avg_delay_time_all', 0),
                    'avg_speed_all': final_stats.get('avg_speed_all', 0),
                    'vehicles_affected_by_crossings': final_stats.get('vehicles_affected_by_crossings', 0),
                    'avg_crossing_delay': final_stats.get('avg_crossing_delay', 0),
                    'avg_travel_time_rerouted': final_stats.get('avg_travel_time_rerouted', 0),
                    'avg_travel_time_non_rerouted': final_stats.get('avg_travel_time_non_rerouted', 0),
                    'avg_delay_rerouted': final_stats.get('avg_delay_rerouted', 0),
                    'avg_delay_non_rerouted': final_stats.get('avg_delay_non_rerouted', 0),
                    'avg_speed_rerouted': final_stats.get('avg_speed_rerouted', 0),
                    'avg_speed_non_rerouted': final_stats.get('avg_speed_non_rerouted', 0),
                    'route_efficiency_rerouted': final_stats.get('route_efficiency_rerouted', 1.0),
                    'route_efficiency_non_rerouted': final_stats.get('route_efficiency_non_rerouted', 1.0),
                    'crossing_stats': {}
                }
            }
            
            # Convert sets to lists for JSON serialization
            for crossing_id, crossing_data in final_stats.get('crossing_stats', {}).items():
                stats_to_save['congestion_metrics']['crossing_stats'][crossing_id] = {
                    'vehicles_affected': len(crossing_data.get('vehicles_affected', set())) if isinstance(crossing_data.get('vehicles_affected'), set) else crossing_data.get('vehicles_affected', 0),
                    'max_queue_length': crossing_data.get('max_queue_length', 0),
                    'total_delay': crossing_data.get('total_delay', 0),
                    'avg_delay_per_vehicle': crossing_data.get('avg_delay_per_vehicle', 0),
                    'duration': crossing_data.get('duration', 0)
                }
            
            # Save statistics to JSON file
            stats_filename = f"stats_{'baseline' if args.baseline else 'rerouting'}.json"
            with open(stats_filename, 'w') as f:
                json.dump(stats_to_save, f, indent=2)
            print(f"\n[OK] Statistics saved to: {stats_filename}")
            
            # Generate visualizations
            print("\nGenerating visualizations...")
            visualizer = MetricsVisualizer()
            
            if args.baseline:
                # Save baseline stats for later comparison
                visualizer.plot_rerouting_statistics(final_stats, "baseline_rerouting_stats.png")
                if final_stats.get('crossing_stats'):
                    visualizer.plot_crossing_impact(final_stats.get('crossing_stats'), "baseline_crossing_impact.png")
                print("[INFO] Baseline plots generated. Run rerouting mode and both will be compared.")
            else:
                # Try to load baseline stats for comparison
                baseline_file = "stats_baseline.json"
                if os.path.exists(baseline_file):
                    with open(baseline_file, 'r') as f:
                        baseline_stats = json.load(f)
                    print("[OK] Found baseline statistics - creating comparison plots...")
                    visualizer.plot_comparison(baseline_stats, stats_to_save)
                else:
                    print("[INFO] No baseline statistics found. Run with --baseline first for comparison.")
                
                # Generate rerouting-specific plots
                visualizer.plot_rerouting_statistics(final_stats, "rerouting_stats.png")
                if final_stats.get('crossing_stats'):
                    visualizer.plot_crossing_impact(final_stats.get('crossing_stats'), "rerouting_crossing_impact.png")
            
            print(f"[OK] All visualizations saved to: metrics_plots/")
            
        except Exception as e:
            print(f"[WARNING] Could not generate visualizations: {e}")
            import traceback
            traceback.print_exc()
        
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
        stats = controller.get_statistics()
        print(f"\nPartial Statistics:")
        print(f"  - Vehicles Rerouted: {stats['reroute_count']}")
        print(f"  - Active Crossings: {stats['active_crossings']}")
        
    except Exception as e:
        print(f"\nError during simulation: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Close simulation
        print("\nClosing simulation...")
        controller.close()
        print("[OK] Simulation closed")


if __name__ == "__main__":
    main()

