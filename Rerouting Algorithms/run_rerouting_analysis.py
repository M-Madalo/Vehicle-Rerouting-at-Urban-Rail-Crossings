"""
Rerouting Analysis Script
Runs SUMO simulation WITH rerouting enabled, collects performance metrics including LOS.
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


def run_rerouting_analysis(
    seed: Optional[int] = None,
    sumo_binary: Optional[str] = None,
    simulation_end_time: float = 3600.0,
    generate_visuals: bool = True,
) -> dict:
    """Run rerouting scenario analysis with rerouting enabled."""

    # Configuration
    SUMO_CONFIG = "4thave.sumocfg"
    SIMULATION_END_TIME = simulation_end_time
    PRINT_STATS_INTERVAL = 60
    output_suffix = f"_seed{seed}" if seed is not None else ""

    print("=" * 70)
    header = "REROUTING ANALYSIS - Dynamic Rerouting ENABLED"
    if seed is not None:
        header += f" (seed={seed})"
    print(header)
    print("Performance Metrics Collection with LOS Calculation")
    print("=" * 70)
    print()

    if not os.path.exists(SUMO_CONFIG):
        print(f"Error: SUMO configuration file '{SUMO_CONFIG}' not found!")
        sys.exit(1)

    # Initialize enhanced Dijkstra algorithm
    print("Initializing Enhanced Dijkstra Algorithm...")
    dijkstra = EnhancedDijkstra(network_graph={})
    print("[OK] Enhanced Dijkstra initialized")
    print()

    # Initialize SUMO controller with rerouting ENABLED (use defaults)
    print("Initializing SUMO Controller (Rerouting ENABLED)...")
    controller = SUMOController(SUMO_CONFIG, dijkstra, enable_rerouting=True)
    controller.sumo_binary = sumo_binary or "sumo-gui"
    if seed is not None:
        controller.sumo_additional_args.append("--seed")
        controller.sumo_additional_args.append(str(seed))
    print("[OK] SUMO Controller initialized")
    print("[INFO] Running in REROUTING mode - vehicles will be rerouted around train crossings")
    print("[INFO] Metrics will be collected for performance analysis")
    if controller.sumo_binary == "sumo-gui":
        print("[INFO] SUMO GUI will open - you can watch the simulation")
    else:
        print("[INFO] Running headless (sumo)")
    print()

    # Initialize metrics collector
    print("Initializing Metrics Collector...")
    metrics_collector = MetricsCollector()
    print("[OK] Metrics Collector initialized")
    print()

    final_stats = {}

    try:
        # Start simulation
        print("Starting SUMO simulation...")
        try:
            controller.start_simulation()
            print("[OK] Simulation started")
            if controller.sumo_binary == "sumo-gui":
                print("[INFO] Waiting for SUMO GUI to initialize...")
                time.sleep(2.0)
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
        print("[INFO] Vehicles will be automatically rerouted when train crossings are detected")
        print()

        start_time = time.time()
        current_sim_time = 0.0
        last_stats_time = 0.0

        while current_sim_time < SIMULATION_END_TIME:
            controller.run_step(current_sim_time)

            # Collect metrics
            active_crossings_list = list(controller.active_crossings.values())
            blocked_edges = set()
            for crossing in active_crossings_list:
                blocked_edges.update(crossing.edge_ids)

            metrics_collector.update_vehicle_metrics(
                current_sim_time, active_crossings_list, blocked_edges,
                controller.train_vehicles, controller.vehicle_routes,
            )

            vehicles_to_check = [v for v in traci.vehicle.getIDList()
                                 if v not in controller.train_vehicles]
            metrics_collector.update_crossing_metrics(
                current_sim_time, active_crossings_list, vehicles_to_check,
                controller.vehicle_routes,
            )

            for veh_id in vehicles_to_check:
                metrics_collector.total_vehicles_seen.add(veh_id)

            metrics_collector.unique_vehicles_rerouted = controller.unique_vehicles_rerouted.copy()

            # Periodic statistics
            if current_sim_time - last_stats_time >= PRINT_STATS_INTERVAL:
                stats = metrics_collector.calculate_metrics()
                print(f"\n[{current_sim_time:.1f}s] Performance Statistics:")
                print(f"  Active vehicles: {len(traci.vehicle.getIDList())}")
                print(f"  Active crossings: {len(active_crossings_list)}")
                print(f"  Vehicles rerouted: {len(controller.unique_vehicles_rerouted)}")
                print(f"  Reroute operations: {controller.reroute_count}")
                if stats:
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

        print("Calculating final metrics...")
        final_stats = metrics_collector.calculate_metrics()

        final_stats['rerouting_stats'] = {
            'total_vehicles_seen': len(metrics_collector.total_vehicles_seen),
            'unique_vehicles_rerouted': len(controller.unique_vehicles_rerouted),
            'reroute_count': controller.reroute_count,
            'reroute_failures': getattr(controller, 'reroute_failures', 0),
        }
        final_stats['total_vehicles_seen'] = len(metrics_collector.total_vehicles_seen)

        # Save statistics JSON
        output_file = f"stats_rerouting{output_suffix}.json"
        print(f"Saving statistics to {output_file}...")
        with open(output_file, 'w') as f:
            json.dump(final_stats, f, indent=2)
        print(f"[OK] Statistics saved to {output_file}")

        # Print summary (rerouting results only)
        print("\n" + "=" * 70)
        print("REROUTING ANALYSIS SUMMARY")
        print("=" * 70)
        if final_stats:
            print(f"\nOverall Performance:")
            print(f"  Total vehicles: {final_stats.get('rerouting_stats', {}).get('total_vehicles_seen', 0)}")
            print(f"  Vehicles rerouted: {final_stats.get('rerouting_stats', {}).get('unique_vehicles_rerouted', 0)}")
            print(f"  Total reroute operations: {final_stats.get('rerouting_stats', {}).get('reroute_count', 0)}")
            print(f"\n--- Delay results ---")
            print(f"  Average delay (all vehicles): {final_stats.get('avg_delay_time_all', 0):.2f} s")
            print(f"  Average speed: {final_stats.get('avg_speed_all', 0)*2.237:.2f} mph")
            print(f"  Level of Service (LOS): {final_stats.get('los_all', 'N/A')}")
            print(f"  Non-rerouted avg delay: {final_stats.get('avg_delay_non_rerouted', 0):.2f} s  (LOS {final_stats.get('los_non_rerouted', 'N/A')})")
            print(f"  Rerouted avg delay: {final_stats.get('avg_delay_rerouted', 0):.2f} s  (LOS {final_stats.get('los_rerouted', 'N/A')})")

        # Generate visualizations (rerouting plots only)
        if generate_visuals:
            print("\nGenerating visualizations...")
            visualizer = MetricsVisualizer()

            visualizer.plot_baseline_metrics(
                final_stats, f"rerouting_metrics{output_suffix}.png",
                title="Dijkstra's Grade-Crossing-Aware Rerouting Performance Analysis",
                is_rerouting=True,
            )

            crossing_stats = final_stats.get('crossing_stats', {})
            if crossing_stats:
                visualizer.plot_crossing_impact(crossing_stats, f"rerouting_crossing_impact{output_suffix}.png")

            visualizer.plot_rerouting_statistics(
                final_stats.get('rerouting_stats', {}), f"rerouting_statistics{output_suffix}.png"
            )

        elapsed_time = time.time() - start_time
        print(f"\n[OK] Analysis complete in {elapsed_time:.1f} seconds")

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

    return final_stats


def _parse_seeds(seeds_arg: Optional[str], seed_arg: Optional[int]) -> List[Optional[int]]:
    """Return list of seeds. Default [None] = no seed."""
    if seeds_arg:
        cleaned = seeds_arg.replace(",", " ").split()
        return [int(x) for x in cleaned if x.strip()]
    if seed_arg is not None:
        return [seed_arg]
    return [None]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run rerouting analysis (single or multiple seeds).")
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

    # Multi-seed: default to headless, no visuals
    default_binary = args.sumo_binary
    if len(seeds) > 1:
        if default_binary is None:
            default_binary = "sumo"
        generate_visuals = False

    # ── Capture all terminal output for .md save ──
    class _TeeLogger:
        """Write to both the real terminal and an internal buffer."""
        def __init__(self, stream):
            self._stream = stream
            self._lines: list = []
        def write(self, msg):
            self._stream.write(msg)
            self._lines.append(msg)
        def flush(self):
            self._stream.flush()
        def get_text(self) -> str:
            return "".join(self._lines)

    _original_stdout = sys.stdout
    _tee = _TeeLogger(_original_stdout)
    sys.stdout = _tee

    all_results = []
    for s in seeds:
        stats = run_rerouting_analysis(
            seed=s,
            sumo_binary=default_binary,
            simulation_end_time=end_time,
            generate_visuals=generate_visuals,
        )
        if stats is not None:
            all_results.append((s, stats))

    if len(all_results) > 1:
        print("\n" + "=" * 70)
        print("DELAY RESULTS (multi-seed summary)")
        print("=" * 70)
        print(f"{'Seed':<8} {'Avg delay (s)':<14} {'LOS':<6} {'Rerouted':<10} {'Reroute ops':<12}")
        print("-" * 54)
        for seed_val, st in all_results:
            delay = st.get("avg_delay_time_all", 0)
            los = st.get("los_all", "N/A")
            n_rer = st.get("rerouting_stats", {}).get("unique_vehicles_rerouted", 0)
            n_ops = st.get("rerouting_stats", {}).get("reroute_count", 0)
            print(f"{seed_val:<8} {delay:<14.2f} {los:<6} {n_rer:<10} {n_ops:<12}")
        avg_delay = sum(st.get("avg_delay_time_all", 0) for _, st in all_results) / len(all_results)
        print("-" * 54)
        print(f"Mean avg delay across seeds: {avg_delay:.2f} s")
        print("=" * 70)

    # ── Save captured output as markdown ──
    sys.stdout = _original_stdout
    seed_labels = [str(s) for s in seeds]
    md_filename = f"Rerouting_Terminal_Output_Seeds_{'_'.join(seed_labels)}.md"
    captured = _tee.get_text()
    with open(md_filename, "w", encoding="utf-8") as f:
        f.write(f"# Rerouting analysis – full terminal output\n\n")
        f.write(f"**Seeds:** {', '.join(seed_labels)}\n\n")
        f.write("---\n\n```\n")
        f.write(captured)
        f.write("\n```\n")
    print(f"\n[OK] Terminal output saved to {md_filename}")
