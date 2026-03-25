"""
Baseline Analysis Script
Runs SUMO simulation without rerouting, collects performance metrics including LOS,
and generates visualizations.
"""

import sys
import os
import time
import traci
import argparse
from typing import List, Optional

from enhanced_dijkstra import EnhancedDijkstra
from sumo_controller import SUMOController
from metrics_collector import MetricsCollector
from visualize_metrics import MetricsVisualizer


def run_baseline_analysis(
    seed: Optional[int] = None,
    sumo_binary: str = "sumo",
    simulation_end_time: float = 3600,
    generate_visuals: bool = True,
    **_unused,
) -> Optional[dict]:
    """Run baseline scenario analysis without rerouting. Returns statistics dict on success."""
    
    # Configuration
    SUMO_CONFIG = "4thave.sumocfg"
    SIMULATION_END_TIME = simulation_end_time if simulation_end_time is not None else 3600  # seconds
    STEP_LENGTH = 1.0  # 1 second per step
    PRINT_STATS_INTERVAL = 60  # Print statistics every 60 seconds
    
    print("=" * 70)
    print("BASELINE ANALYSIS - No Rerouting")
    print("Performance Metrics Collection with LOS Calculation")
    print("=" * 70)
    print()
    
    # Check if SUMO config exists
    if not os.path.exists(SUMO_CONFIG):
        print(f"Error: SUMO configuration file '{SUMO_CONFIG}' not found!")
        print("Please ensure the file exists in the current directory.")
        sys.exit(1)
    
    # Initialize enhanced Dijkstra algorithm (not used in baseline, but required by controller)
    print("Initializing Enhanced Dijkstra Algorithm...")
    dijkstra = EnhancedDijkstra(network_graph={})
    print("[OK] Enhanced Dijkstra initialized")
    print()
    
    # Initialize SUMO controller with rerouting DISABLED
    print("Initializing SUMO Controller (Rerouting DISABLED)...")
    controller = SUMOController(SUMO_CONFIG, dijkstra, enable_rerouting=False)
    controller.sumo_binary = sumo_binary or "sumo"
    if seed is not None:
        # Pass seed through to SUMO for reproducible baseline runs
        controller.sumo_additional_args.extend(["--seed", str(seed)])
    print("[OK] SUMO Controller initialized")
    print("[INFO] Running in BASELINE mode - no rerouting will occur")
    print("[INFO] Metrics will be collected for performance analysis")
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
        print("Note: Trains are scheduled to depart at:")
        print("  - train_1: 300 seconds")
        print("  - train_2: 480 seconds")
        print()
        
        start_time = time.time()
        current_sim_time = 0.0
        last_stats_time = 0.0
        
        while current_sim_time < SIMULATION_END_TIME:
            # Run one simulation step (this also calls detect_train_crossings internally)
            controller.run_step(current_sim_time)
            
            # Collect metrics
            # Get active crossings from the controller (already detected in run_step)
            active_crossings_list = list(controller.active_crossings.values())
            blocked_edges = set()
            for crossing in active_crossings_list:
                blocked_edges.update(crossing.edge_ids)
            
            all_vehicles = traci.vehicle.getIDList()
            vehicles_to_check = [v for v in all_vehicles if v not in controller.train_vehicles]
            
            # Update metrics collector
            metrics_collector.update_vehicle_metrics(
                current_sim_time, 
                active_crossings_list, 
                blocked_edges,
                controller.train_vehicles,
                controller.vehicle_routes
            )
            metrics_collector.update_crossing_metrics(
                current_sim_time,
                active_crossings_list,
                vehicles_to_check,
                controller.vehicle_routes
            )
            
            # Track unique vehicles
            for veh_id in vehicles_to_check:
                metrics_collector.total_vehicles_seen.add(veh_id)
            
            # Update simulation time
            current_sim_time += STEP_LENGTH
            
            # Print statistics periodically
            if current_sim_time - last_stats_time >= PRINT_STATS_INTERVAL:
                stats = metrics_collector.get_statistics()
                actual_vehicles = len(traci.vehicle.getIDList())
                
                print(f"\n[Time: {current_sim_time:.1f}s] BASELINE Statistics:")
                print(f"  - Vehicles in SUMO: {actual_vehicles}")
                print(f"  - Current Vehicles Tracked: {stats['total_vehicles']}")
                print(f"  - Total Unique Vehicles Seen: {stats['total_vehicles_seen']}")
                print(f"  - Active Train Crossings: {stats['active_crossings']}")
                print(f"  - Train Vehicles: {len(controller.train_vehicles)}")
                
                if actual_vehicles == 0:
                    print(f"  - [WARNING] No vehicles in simulation yet - check depart times in routes file")
                
                # Performance Metrics
                if stats.get('avg_travel_time_all', 0) > 0:
                    print(f"\n  PERFORMANCE METRICS:")
                    print(f"  - Average Travel Time: {stats.get('avg_travel_time_all', 0):.2f} s")
                    print(f"  - Average Delay Time: {stats.get('avg_delay_time_all', 0):.2f} s")
                    print(f"  - Average Speed: {stats.get('avg_speed_all', 0):.2f} m/s ({stats.get('avg_speed_all', 0)*2.237:.2f} mph)")
                    print(f"  - Free-Flow Speed: {stats.get('avg_free_flow_speed_all', 0):.2f} m/s ({stats.get('avg_free_flow_speed_all', 0)*2.237:.2f} mph)")
                    print(f"  - Speed Ratio: {stats.get('speed_ratio_all', 0):.2%}")
                    print(f"  - Route Efficiency: {stats.get('route_efficiency_all', 1.0):.2%}")
                    
                    # LOS Information
                    los = stats.get('los_all', 'N/A')
                    print(f"\n  LEVEL OF SERVICE (LOS):")
                    print(f"  - Overall LOS: {los}")
                    if los != 'N/A':
                        los_descriptions = {
                            'A': 'Free flow - Excellent',
                            'B': 'Reasonably free flow - Good',
                            'C': 'Stable flow - Acceptable',
                            'D': 'Approaching unstable - Tolerable',
                            'E': 'Unstable flow - Poor',
                            'F': 'Forced flow/Breakdown - Very Poor'
                        }
                        print(f"    Description: {los_descriptions.get(los, 'Unknown')}")
                
                # Crossing Metrics
                if stats.get('crossing_stats'):
                    print(f"\n  CROSSING-SPECIFIC METRICS:")
                    for crossing_id, crossing_data in stats['crossing_stats'].items():
                        vehicles_affected = crossing_data.get('vehicles_affected', 0)
                        print(f"  - {crossing_id}:")
                        print(f"    Vehicles Affected: {vehicles_affected}")
                        print(f"    Max Queue Length: {crossing_data.get('max_queue_length', 0)}")
                        print(f"    Avg Delay per Vehicle: {crossing_data.get('avg_delay_per_vehicle', 0):.2f} s")
                        print(f"    Duration: {crossing_data.get('duration', 0):.2f} s")
                        if vehicles_affected == 0:
                            # Debug info when no vehicles affected
                            active_crossings_list = list(controller.active_crossings.values())
                            for crossing in active_crossings_list:
                                if crossing.crossing_id == crossing_id:
                                    print(f"    [DEBUG] Blocked Edges: {len(crossing.edge_ids)} edges")
                                    print(f"    [DEBUG] Current Vehicles in Simulation: {len(traci.vehicle.getIDList())}")
                                    break
                
                last_stats_time = current_sim_time
        
        # Final statistics
        print("\n" + "=" * 70)
        print("SIMULATION COMPLETE")
        print("=" * 70)
        
        elapsed_time = time.time() - start_time
        print(f"Simulation Time: {SIMULATION_END_TIME} seconds")
        print(f"Real Time Elapsed: {elapsed_time:.2f} seconds")
        print()
        
        final_stats = metrics_collector.get_statistics()
        
        print("Final BASELINE Statistics:")
        print(f"  - Total Vehicles Tracked: {final_stats['total_vehicles']}")
        print(f"  - Total Unique Vehicles Seen: {final_stats['total_vehicles_seen']}")
        print(f"  - Active Crossings: {final_stats['active_crossings']}")
        
        # Final Performance Metrics
        if final_stats.get('avg_travel_time_all', 0) > 0:
            print(f"\n  FINAL PERFORMANCE METRICS:")
            print(f"  - Average Travel Time: {final_stats.get('avg_travel_time_all', 0):.2f} s")
            print(f"  - Average Delay Time: {final_stats.get('avg_delay_time_all', 0):.2f} s")
            print(f"  - Average Speed: {final_stats.get('avg_speed_all', 0):.2f} m/s ({final_stats.get('avg_speed_all', 0)*2.237:.2f} mph)")
            print(f"  - Free-Flow Speed: {final_stats.get('avg_free_flow_speed_all', 0):.2f} m/s ({final_stats.get('avg_free_flow_speed_all', 0)*2.237:.2f} mph)")
            print(f"  - Speed Ratio: {final_stats.get('speed_ratio_all', 0):.2%}")
            print(f"  - Route Efficiency: {final_stats.get('route_efficiency_all', 1.0):.2%}")
            print(f"  - Delay per Vehicle: {final_stats.get('delay_per_vehicle_all', 0):.2f} s")
            
            # Final LOS
            los = final_stats.get('los_all', 'N/A')
            print(f"\n  FINAL LEVEL OF SERVICE (LOS):")
            print(f"  - Overall LOS: {los}")
            if los != 'N/A':
                los_descriptions = {
                    'A': 'Free flow - Excellent',
                    'B': 'Reasonably free flow - Good',
                    'C': 'Stable flow - Acceptable',
                    'D': 'Approaching unstable - Tolerable',
                    'E': 'Unstable flow - Poor',
                    'F': 'Forced flow/Breakdown - Very Poor'
                }
                print(f"    Description: {los_descriptions.get(los, 'Unknown')}")
        
        # Save statistics
        print("\nSaving statistics and generating visualizations...")
        stats_suffix = f"_seed{seed}" if seed is not None else ""
        stats_filename = f"stats_baseline{stats_suffix}.json"
        saved_data = metrics_collector.save_metrics(
            stats_filename, 
            SIMULATION_END_TIME, 
            elapsed_time, 
            mode="baseline"
        )
        print(f"[OK] Statistics saved to: {stats_filename}")
        
        # Generate visualizations (usually only for single runs, not big multi-seed batches)
        if generate_visuals:
            print("\nGenerating visualizations...")
            visualizer = MetricsVisualizer()
            visualizer.plot_baseline_metrics(final_stats, f"baseline_metrics{stats_suffix}.png")
            print(f"[OK] Visualizations saved to: metrics_plots/")
        
        print("\n" + "=" * 70)
        print("BASELINE ANALYSIS COMPLETE")
        print("=" * 70)
        print(f"\nResults saved to:")
        print(f"  - {stats_filename}")
        if generate_visuals:
            print(f"  - metrics_plots/baseline_metrics{stats_suffix}.png")
        print()
        
        traci.close()
        return final_stats
        
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
        traci.close()
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Error during simulation: {e}")
        import traceback
        traceback.print_exc()
        traci.close()
        sys.exit(1)


def _parse_seeds(seeds_arg: Optional[str], seed_arg: Optional[int]) -> List[Optional[int]]:
    """Return list of seeds from --seeds or single --seed. Default [None] = no seed."""
    if seeds_arg:
        cleaned = seeds_arg.replace(",", " ").split()
        return [int(x) for x in cleaned if x.strip()]
    if seed_arg is not None:
        return [seed_arg]
    return [None]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run baseline analysis (single or multiple seeds).")
    parser.add_argument("--seed", type=int, default=None, help="Single SUMO random seed.")
    parser.add_argument(
        "--seeds",
        type=str,
        default=None,
        help="Comma/space separated seeds, e.g. 1001,1002,1003.",
    )
    parser.add_argument(
        "--sumo-binary",
        type=str,
        choices=["sumo", "sumo-gui"],
        default="sumo",
        help="SUMO binary (default: sumo for batch).",
    )
    parser.add_argument(
        "--end-time",
        type=float,
        default=3600,
        help="Simulation end time in seconds.",
    )
    parser.add_argument(
        "--no-visuals",
        action="store_true",
        help="Skip generating plots (faster for multi-seed runs).",
    )
    args = parser.parse_args()

    seeds = _parse_seeds(args.seeds, args.seed)
    generate_visuals = not args.no_visuals

    for s in seeds:
        run_baseline_analysis(
            seed=s,
            sumo_binary=args.sumo_binary,
            simulation_end_time=args.end_time,
            generate_visuals=generate_visuals,
        )

