"""
Naive Rerouting Analysis Script
Runs SUMO simulation with naive rerouting enabled: any vehicle near a train crossing
is told to reroute, regardless of whether its route intersects the crossing.
Collects performance metrics including LOS and generates visualizations.
"""

import sys
import os
import time
import traci
import json
import argparse
from typing import List, Optional

from enhanced_dijkstra import EnhancedDijkstra
from sumo_controller import SUMOController
from metrics_collector import MetricsCollector
from visualize_metrics import MetricsVisualizer
try:
    from export_to_excel import ExcelExporter
except ImportError:
    ExcelExporter = None  # optional: skip Excel export if module not present


def run_naive_rerouting_analysis(
    seed: Optional[int] = None,
    sumo_binary: Optional[str] = None,
    simulation_end_time: float = 3600,
    generate_visuals: bool = True,
) -> Optional[dict]:
    """Run naive rerouting scenario analysis with rerouting enabled. Returns statistics dict on success."""

    # Configuration
    SUMO_CONFIG = "4thave.sumocfg"
    SIMULATION_END_TIME = simulation_end_time
    STEP_LENGTH = 1.0  # 1 second per step
    PRINT_STATS_INTERVAL = 60  # Print statistics every 60 seconds
    NAIVE_TRIGGER_HOPS = 5  # Edge-to-edge hops around crossings
    output_suffix = f"_seed{seed}" if seed is not None else ""

    print("=" * 70)
    header = "NAIVE REROUTING ANALYSIS - Crossing-Triggered Rerouting"
    if seed is not None:
        header += f" (seed={seed})"
    print(header)
    print("Vehicles near crossings reroute regardless of route relevance")
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

    # Initialize SUMO controller with naive rerouting
    print("Initializing SUMO Controller (Naive Rerouting ENABLED)...")
    controller = SUMOController(
        SUMO_CONFIG,
        dijkstra,
        enable_rerouting=True,
        rerouting_strategy="naive",
        naive_hops=NAIVE_TRIGGER_HOPS
    )
    controller.sumo_binary = sumo_binary or "sumo-gui"
    if seed is not None:
        controller.sumo_additional_args.extend(["--seed", str(seed)])
    print("[OK] SUMO Controller initialized")
    print("[INFO] Running in NAIVE REROUTING mode - vehicles near crossings will be rerouted")
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
        print("[INFO] Naive rerouting triggers for any vehicle near active crossings")
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

            # Update vehicle metrics
            metrics_collector.update_vehicle_metrics(
                current_sim_time,
                active_crossings_list,
                blocked_edges,
                controller.train_vehicles,
                controller.vehicle_routes
            )

            # Update crossing metrics
            vehicles_to_check = [v for v in traci.vehicle.getIDList()
                               if v not in controller.train_vehicles]
            metrics_collector.update_crossing_metrics(
                current_sim_time,
                active_crossings_list,
                vehicles_to_check,
                controller.vehicle_routes
            )

            # Track unique vehicles seen (for total_vehicles_seen)
            for veh_id in vehicles_to_check:
                metrics_collector.total_vehicles_seen.add(veh_id)

            # Track rerouted vehicles for metrics
            metrics_collector.unique_vehicles_rerouted = controller.unique_vehicles_rerouted.copy()

            # Print periodic statistics
            if current_sim_time - last_stats_time >= PRINT_STATS_INTERVAL:
                stats = metrics_collector.calculate_metrics()
                print(f"\n[{current_sim_time:.1f}s] Performance Statistics:")
                print(f"  Active vehicles: {len(traci.vehicle.getIDList())}")
                print(f"  Active crossings: {len(active_crossings_list)}")
                print(f"  Vehicles rerouted: {len(controller.unique_vehicles_rerouted)}")
                print(f"  Reroute operations: {controller.reroute_count}")
                if stats:
                    print(f"  Avg travel time: {stats.get('avg_travel_time_all', 0):.1f}s")
                    print(f"  Avg delay: {stats.get('avg_delay_time_all', 0):.1f}s")
                    print(f"  Avg speed: {stats.get('avg_speed_all', 0)*2.237:.1f} mph")
                    print(f"  LOS: {stats.get('los_all', 'N/A')}")
                last_stats_time = current_sim_time

            # Advance simulation time (SUMO advanced inside controller.run_step)
            current_sim_time += STEP_LENGTH

        # Final statistics
        print("\n" + "=" * 70)
        print("SIMULATION COMPLETE")
        print("=" * 70)
        print()

        # Calculate final metrics
        print("Calculating final metrics...")
        final_stats = metrics_collector.calculate_metrics()

        # Add rerouting statistics
        final_stats['rerouting_stats'] = {
            'total_vehicles_seen': len(metrics_collector.total_vehicles_seen),
            'unique_vehicles_rerouted': len(controller.unique_vehicles_rerouted),
            'reroute_count': controller.reroute_count,
            'reroute_failures': getattr(controller, 'reroute_failures', 0)
        }

        # Ensure total_vehicles_seen is in final_stats at top level (for visualization)
        final_stats['total_vehicles_seen'] = len(metrics_collector.total_vehicles_seen)

        # Save statistics
        output_file = f"stats_naive_rerouting{output_suffix}.json"
        print(f"Saving statistics to {output_file}...")
        with open(output_file, 'w') as f:
            json.dump(final_stats, f, indent=2)
        print(f"[OK] Statistics saved to {output_file}")

        # Print summary
        print("\n" + "=" * 70)
        print("NAIVE REROUTING ANALYSIS SUMMARY")
        print("=" * 70)
        if final_stats:
            print(f"\nOverall Performance:")
            print(f"  Total vehicles: {final_stats.get('rerouting_stats', {}).get('total_vehicles_seen', 0)}")
            print(f"  Vehicles rerouted: {final_stats.get('rerouting_stats', {}).get('unique_vehicles_rerouted', 0)}")
            print(f"  Total reroute operations: {final_stats.get('rerouting_stats', {}).get('reroute_count', 0)}")
            print(f"  Average travel time: {final_stats.get('avg_travel_time_all', 0):.1f} seconds")
            print(f"  Average delay: {final_stats.get('avg_delay_time_all', 0):.1f} seconds")
            print(f"  Average speed: {final_stats.get('avg_speed_all', 0)*2.237:.1f} mph")
            print(f"  Speed ratio: {final_stats.get('speed_ratio_all', 0)*100:.1f}%")
            print(f"  Route efficiency: {final_stats.get('route_efficiency_all', 1.0)*100:.1f}%")
            print(f"  Level of Service (LOS): {final_stats.get('los_all', 'N/A')}")
            print(f"  Vehicles affected by crossings: {final_stats.get('vehicles_affected_by_crossings', 0)}")
            print(f"  Average crossing delay: {final_stats.get('avg_crossing_delay_all', 0):.1f} seconds")

        # Generate visualizations (only when requested, e.g. single-seed runs)
        if generate_visuals:
            print("\nGenerating visualizations...")
            visualizer = MetricsVisualizer()

            visualizer.plot_baseline_metrics(
                final_stats,
                f"naive_rerouting_metrics{output_suffix}.png",
                title="Naive Crossing-Triggered Rerouting Performance Analysis",
                is_rerouting=True
            )

            crossing_stats = final_stats.get('crossing_stats', {})
            if crossing_stats:
                visualizer.plot_crossing_impact(crossing_stats, f"naive_rerouting_crossing_impact{output_suffix}.png")

            visualizer.plot_rerouting_statistics(
                final_stats.get('rerouting_stats', {}), f"naive_rerouting_statistics{output_suffix}.png"
            )

            baseline_file = f"stats_baseline{output_suffix}.json"
            if not os.path.exists(baseline_file):
                baseline_file = "stats_baseline.json"
            if os.path.exists(baseline_file):
                print(f"\nComparing with baseline results from {baseline_file}...")
                with open(baseline_file, 'r') as f:
                    baseline_stats = json.load(f)
                stats_for_compare = baseline_stats.get('statistics', baseline_stats)
                visualizer.plot_comparison(stats_for_compare, final_stats, f"baseline_vs_naive_rerouting_comparison{output_suffix}.png")
                print("[OK] Comparison plot generated")

            if ExcelExporter is not None:
                print("\nCreating Excel results file...")
                try:
                    excel_exporter = ExcelExporter()
                    excel_stats = {
                        'mode': 'naive_rerouting',
                        'simulation_time': SIMULATION_END_TIME,
                        'real_time': time.time() - start_time,
                        'statistics': final_stats
                    }
                    excel_exporter.create_excel_for_analysis(
                        "naive_rerouting",
                        excel_stats,
                        f"results_naive_rerouting{output_suffix}.xlsx"
                    )
                    print("[OK] Excel file created")
                except Exception as ex:
                    print(f"[WARNING] Excel export skipped: {ex}")
            else:
                print("\n[INFO] Excel export skipped (export_to_excel module not available)")

        elapsed_time = time.time() - start_time
        print(f"\n[OK] Analysis complete in {elapsed_time:.1f} seconds")
        if generate_visuals:
            print(f"[OK] Visualizations saved to metrics_plots/ directory")

        return final_stats

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

    return None


def _parse_seeds(seeds_arg: Optional[str], seed_arg: Optional[int]) -> List[Optional[int]]:
    """Return list of seeds from --seeds or single --seed. Default [None] = no seed."""
    if seeds_arg:
        cleaned = seeds_arg.replace(",", " ").split()
        return [int(x) for x in cleaned if x.strip()]
    if seed_arg is not None:
        return [seed_arg]
    return [None]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run naive rerouting analysis (single or multiple seeds).")
    parser.add_argument("--seed", type=int, default=None, help="Single SUMO random seed.")
    parser.add_argument("--seeds", type=str, default=None, help="Comma/space separated seeds, e.g. 1001,1002,1003.")
    parser.add_argument("--sumo-binary", type=str, choices=["sumo", "sumo-gui"], default=None,
                        help="SUMO binary (defaults to sumo-gui; use sumo for batch).")
    parser.add_argument("--end-time", type=float, default=3600, help="Simulation end time in seconds.")
    parser.add_argument("--no-visuals", action="store_true", help="Skip generating plots (faster for batch).")
    args = parser.parse_args()

    seeds = _parse_seeds(args.seeds, args.seed)
    end_time = args.end_time
    generate_visuals = not args.no_visuals
    default_binary = args.sumo_binary
    if len(seeds) > 1:
        if default_binary is None:
            default_binary = "sumo"
        generate_visuals = False

    for s in seeds:
        run_naive_rerouting_analysis(
            seed=s,
            sumo_binary=default_binary,
            simulation_end_time=end_time,
            generate_visuals=generate_visuals,
        )
