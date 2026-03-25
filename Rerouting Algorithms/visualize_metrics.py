"""
Visualization module for simulation metrics
Creates graphs to visualize congestion, travel time, and rerouting effectiveness
"""

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import json
import os
from typing import Dict, List, Optional
from collections import defaultdict
import numpy as np


class MetricsVisualizer:
    """Creates visualizations from simulation metrics."""
    
    def __init__(self, output_dir: str = "metrics_plots"):
        """Initialize visualizer.
        
        Args:
            output_dir: Directory to save plots
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
    def plot_comparison(self, baseline_stats: Dict, rerouting_stats: Dict, filename: str = "comparison.png"):
        """Create comparison plots between baseline and rerouting modes.
        
        Args:
            baseline_stats: Statistics from baseline run
            rerouting_stats: Statistics from rerouting run
            filename: Output filename
        """
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Baseline vs Rerouting Performance Comparison', fontsize=16, fontweight='bold')
        
        # Extract metrics (support both flat stats and nested congestion_metrics)
        baseline_metrics = baseline_stats.get('congestion_metrics', baseline_stats)
        rerouting_metrics = rerouting_stats.get('congestion_metrics', rerouting_stats)
        
        # 1. Travel Time Comparison
        ax1 = axes[0, 0]
        categories = ['All Vehicles', 'Rerouted', 'Non-Rerouted']
        baseline_times = [
            baseline_metrics.get('avg_travel_time_all', 0),
            baseline_metrics.get('avg_travel_time_rerouted', baseline_metrics.get('avg_travel_time_all', 0)),
            baseline_metrics.get('avg_travel_time_non_rerouted', baseline_metrics.get('avg_travel_time_all', 0))
        ]
        rerouting_times = [
            rerouting_metrics.get('avg_travel_time_all', 0),
            rerouting_metrics.get('avg_travel_time_rerouted', rerouting_metrics.get('avg_travel_time_all', 0)),
            rerouting_metrics.get('avg_travel_time_non_rerouted', rerouting_metrics.get('avg_travel_time_all', 0))
        ]
        
        x = np.arange(len(categories))
        width = 0.35
        ax1.bar(x - width/2, baseline_times, width, label='Baseline', color='#ff7f0e', alpha=0.8)
        ax1.bar(x + width/2, rerouting_times, width, label='Rerouting', color='#2ca02c', alpha=0.8)
        ax1.set_ylabel('Travel Time (seconds)', fontsize=12)
        ax1.set_title('Average Travel Time Comparison', fontsize=13, fontweight='bold')
        ax1.set_xticks(x)
        ax1.set_xticklabels(categories, rotation=15, ha='right')
        ax1.legend()
        ax1.grid(axis='y', alpha=0.3)
        
        # 2. Delay Time Comparison
        ax2 = axes[0, 1]
        baseline_delays = [
            baseline_metrics.get('avg_delay_time_all', 0),
            baseline_metrics.get('avg_delay_rerouted', baseline_metrics.get('avg_delay_time_all', 0)),
            baseline_metrics.get('avg_delay_non_rerouted', baseline_metrics.get('avg_delay_time_all', 0))
        ]
        rerouting_delays = [
            rerouting_metrics.get('avg_delay_time_all', 0),
            rerouting_metrics.get('avg_delay_rerouted', rerouting_metrics.get('avg_delay_time_all', 0)),
            rerouting_metrics.get('avg_delay_non_rerouted', rerouting_metrics.get('avg_delay_time_all', 0))
        ]
        
        ax2.bar(x - width/2, baseline_delays, width, label='Baseline', color='#ff7f0e', alpha=0.8)
        ax2.bar(x + width/2, rerouting_delays, width, label='Rerouting', color='#2ca02c', alpha=0.8)
        ax2.set_ylabel('Delay Time (seconds)', fontsize=12)
        ax2.set_title('Average Delay Time Comparison', fontsize=13, fontweight='bold')
        ax2.set_xticks(x)
        ax2.set_xticklabels(categories, rotation=15, ha='right')
        ax2.legend()
        ax2.grid(axis='y', alpha=0.3)
        
        # 3. Average Speed Comparison
        ax3 = axes[1, 0]
        baseline_speeds = [
            baseline_metrics.get('avg_speed_all', 0) * 2.237,  # Convert m/s to mph
            baseline_metrics.get('avg_speed_rerouted', baseline_metrics.get('avg_speed_all', 0)) * 2.237,
            baseline_metrics.get('avg_speed_non_rerouted', baseline_metrics.get('avg_speed_all', 0)) * 2.237
        ]
        rerouting_speeds = [
            rerouting_metrics.get('avg_speed_all', 0) * 2.237,
            rerouting_metrics.get('avg_speed_rerouted', rerouting_metrics.get('avg_speed_all', 0)) * 2.237,
            rerouting_metrics.get('avg_speed_non_rerouted', rerouting_metrics.get('avg_speed_all', 0)) * 2.237
        ]
        
        ax3.bar(x - width/2, baseline_speeds, width, label='Baseline', color='#ff7f0e', alpha=0.8)
        ax3.bar(x + width/2, rerouting_speeds, width, label='Rerouting', color='#2ca02c', alpha=0.8)
        ax3.set_ylabel('Average Speed (mph)', fontsize=12)
        ax3.set_title('Average Speed Comparison', fontsize=13, fontweight='bold')
        ax3.set_xticks(x)
        ax3.set_xticklabels(categories, rotation=15, ha='right')
        ax3.legend()
        ax3.grid(axis='y', alpha=0.3)
        
        # 4. Improvement Metrics
        ax4 = axes[1, 1]
        if baseline_metrics.get('avg_travel_time_all', 0) > 0:
            travel_improvement = ((baseline_metrics.get('avg_travel_time_all', 0) - 
                                  rerouting_metrics.get('avg_travel_time_all', 0)) / 
                                 baseline_metrics.get('avg_travel_time_all', 1)) * 100
            delay_reduction = ((baseline_metrics.get('avg_delay_time_all', 0) - 
                               rerouting_metrics.get('avg_delay_time_all', 0)) / 
                              baseline_metrics.get('avg_delay_time_all', 1)) * 100 if baseline_metrics.get('avg_delay_time_all', 0) > 0 else 0
            speed_improvement = ((rerouting_metrics.get('avg_speed_all', 0) - 
                                 baseline_metrics.get('avg_speed_all', 0)) / 
                                baseline_metrics.get('avg_speed_all', 1)) * 100 if baseline_metrics.get('avg_speed_all', 0) > 0 else 0
            
            improvements = [travel_improvement, delay_reduction, speed_improvement]
            improvement_labels = ['Travel Time\nImprovement', 'Delay\nReduction', 'Speed\nImprovement']
            colors = ['green' if x > 0 else 'red' for x in improvements]
            
            bars = ax4.bar(improvement_labels, improvements, color=colors, alpha=0.7)
            ax4.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
            ax4.set_ylabel('Improvement (%)', fontsize=12)
            ax4.set_title('Rerouting Effectiveness', fontsize=13, fontweight='bold')
            ax4.grid(axis='y', alpha=0.3)
            
            # Add value labels on bars
            for bar, val in zip(bars, improvements):
                height = bar.get_height()
                ax4.text(bar.get_x() + bar.get_width()/2., height,
                        f'{val:.1f}%',
                        ha='center', va='bottom' if height > 0 else 'top', fontsize=10)
        
        plt.tight_layout()
        output_path = os.path.join(self.output_dir, filename)
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"[OK] Saved comparison plot: {output_path}")
    
    def plot_crossing_impact(self, crossing_stats: Dict, filename: str = "crossing_impact.png"):
        """Plot crossing-specific impact metrics.
        
        Args:
            crossing_stats: Dictionary of crossing statistics
            filename: Output filename
        """
        if not crossing_stats:
            print("[WARNING] No crossing statistics to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Train Crossing Impact Analysis', fontsize=16, fontweight='bold')
        
        crossing_ids = list(crossing_stats.keys())
        vehicles_affected = [crossing_stats[cid].get('vehicles_affected', 0) for cid in crossing_ids]
        max_queues = [crossing_stats[cid].get('max_queue_length', 0) for cid in crossing_ids]
        total_delays = [crossing_stats[cid].get('total_delay', 0) for cid in crossing_ids]
        avg_delays = [crossing_stats[cid].get('avg_delay_per_vehicle', 0) for cid in crossing_ids]
        
        # 1. Vehicles Affected per Crossing
        ax1 = axes[0, 0]
        ax1.bar(range(len(crossing_ids)), vehicles_affected, color='#1f77b4', alpha=0.7)
        ax1.set_ylabel('Number of Vehicles (Per Crossing)', fontsize=12)
        ax1.set_title('Vehicles Affected per Crossing', fontsize=13, fontweight='bold')
        ax1.set_xticks(range(len(crossing_ids)))
        ax1.set_xticklabels(crossing_ids, rotation=15, ha='right')
        ax1.grid(axis='y', alpha=0.3)
        
        # 2. Maximum Queue Length
        ax2 = axes[0, 1]
        ax2.bar(range(len(crossing_ids)), max_queues, color='#ff7f0e', alpha=0.7)
        ax2.set_ylabel('Maximum Queue Length', fontsize=12)
        ax2.set_title('Maximum Queue Length per Crossing', fontsize=13, fontweight='bold')
        ax2.set_xticks(range(len(crossing_ids)))
        ax2.set_xticklabels(crossing_ids, rotation=15, ha='right')
        ax2.grid(axis='y', alpha=0.3)
        
        # 3. Total Delay per Crossing
        ax3 = axes[1, 0]
        ax3.bar(range(len(crossing_ids)), total_delays, color='#d62728', alpha=0.7)
        ax3.set_ylabel('Total Delay (seconds)', fontsize=12)
        ax3.set_title('Total Delay per Crossing', fontsize=13, fontweight='bold')
        ax3.set_xticks(range(len(crossing_ids)))
        ax3.set_xticklabels(crossing_ids, rotation=15, ha='right')
        ax3.grid(axis='y', alpha=0.3)
        
        # 4. Average Delay per Vehicle
        ax4 = axes[1, 1]
        ax4.bar(range(len(crossing_ids)), avg_delays, color='#2ca02c', alpha=0.7)
        ax4.set_ylabel('Average Delay (seconds)', fontsize=12)
        ax4.set_title('Average Delay per Vehicle per Crossing', fontsize=13, fontweight='bold')
        ax4.set_xticks(range(len(crossing_ids)))
        ax4.set_xticklabels(crossing_ids, rotation=15, ha='right')
        ax4.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        output_path = os.path.join(self.output_dir, filename)
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"[OK] Saved crossing impact plot: {output_path}")
    
    def plot_rerouting_statistics(self, stats: Dict, filename: str = "rerouting_stats.png"):
        """Plot rerouting statistics.
        
        Args:
            stats: Statistics dictionary
            filename: Output filename
        """
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('Rerouting Statistics', fontsize=16, fontweight='bold')
        
        # 1. Reroute Rate Pie Chart
        ax1 = axes[0]
        total_vehicles = stats.get('total_vehicles_seen', 0)
        rerouted = stats.get('unique_vehicles_rerouted', 0)
        not_rerouted = total_vehicles - rerouted
        
        if total_vehicles > 0:
            sizes = [rerouted, not_rerouted]
            labels = [f'Rerouted\n({rerouted})', f'Not Rerouted\n({not_rerouted})']
            colors = ['#2ca02c', '#d62728']
            explode = (0.1, 0)  # Explode the rerouted slice
            
            ax1.pie(sizes, explode=explode, labels=labels, colors=colors, autopct='%1.1f%%',
                   shadow=True, startangle=90, textprops={'fontsize': 11})
            ax1.set_title('Vehicle Rerouting Distribution', fontsize=13, fontweight='bold')
        
        # 2. Reroute Operations Bar Chart
        ax2 = axes[1]
        categories = ['Total Vehicles', 'Rerouted Vehicles', 'Reroute Operations']
        values = [
            total_vehicles,
            rerouted,
            stats.get('reroute_count', 0)
        ]
        
        bars = ax2.bar(categories, values, color=['#1f77b4', '#2ca02c', '#ff7f0e'], alpha=0.7)
        ax2.set_ylabel('Count', fontsize=12)
        ax2.set_title('Rerouting Operations Summary', fontsize=13, fontweight='bold')
        ax2.grid(axis='y', alpha=0.3)
        
        # Add value labels on bars
        for bar, val in zip(bars, values):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{int(val)}',
                    ha='center', va='bottom', fontsize=10)
        
        plt.tight_layout()
        output_path = os.path.join(self.output_dir, filename)
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"[OK] Saved rerouting statistics plot: {output_path}")
    
    def plot_baseline_metrics(self, stats: Dict, filename: str = "baseline_metrics.png", 
                             title: str = None, is_rerouting: bool = False):
        """Plot baseline metrics including LOS visualization.
        
        Args:
            stats: Statistics dictionary from baseline run
            filename: Output filename
            title: Custom title for the plot (if None, uses default based on is_rerouting)
            is_rerouting: Whether this is a rerouting scenario (affects default title)
        """
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # Set title based on mode
        if title is None:
            if is_rerouting:
                title = "Dijkstra's Grade-Crossing-Aware Rerouting Performance Analysis"
            else:
                title = 'Baseline Performance Metrics Analysis (No Rerouting)'
        
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        # Removed rerouting status indicator - no longer showing on visualization
        
        # Extract metrics
        avg_travel_time = stats.get('avg_travel_time_all', 0)
        avg_delay = stats.get('avg_delay_time_all', 0)
        avg_speed = stats.get('avg_speed_all', 0) * 2.237  # Convert m/s to mph
        free_flow_speed = stats.get('avg_free_flow_speed_all', 0) * 2.237
        speed_ratio = stats.get('speed_ratio_all', 0)
        route_efficiency = stats.get('route_efficiency_all', 1.0)
        los = stats.get('los_all', 'N/A')
        delay_per_vehicle = stats.get('delay_per_vehicle_all', 0)
        
        # 1. LOS Visualization (Large center plot)
        ax_los = fig.add_subplot(gs[0:2, 1])
        los_levels = ['A', 'B', 'C', 'D', 'E', 'F']
        los_colors = ['#00ff00', '#90ee90', '#ffff00', '#ffa500', '#ff6347', '#8b0000']
        los_values = [90, 70, 50, 40, 30, 0]  # Speed ratio thresholds (%)
        
        # Create LOS gauge/bar
        current_los_idx = los_levels.index(los) if los in los_levels else 5
        los_bar = ax_los.barh(range(len(los_levels)), los_values, 
                             color=los_colors, alpha=0.3, edgecolor='black', linewidth=1)
        
        # Highlight current LOS
        if los in los_levels:
            los_bar[current_los_idx].set_alpha(0.9)
            los_bar[current_los_idx].set_edgecolor('black')
            los_bar[current_los_idx].set_linewidth(3)
        
        ax_los.set_yticks(range(len(los_levels)))
        ax_los.set_yticklabels([f'LOS {level}' for level in los_levels], fontsize=11)
        ax_los.set_xlabel('Speed Ratio Threshold (%)', fontsize=12)
        ax_los.set_title(f'Level of Service (LOS): {los}', fontsize=14, fontweight='bold')
        ax_los.axvline(x=speed_ratio * 100, color='red', linestyle='--', linewidth=2, 
                      label=f'Current: {speed_ratio*100:.1f}%')
        ax_los.legend(fontsize=10)
        ax_los.grid(axis='x', alpha=0.3)
        ax_los.set_xlim(0, 100)
        
        # Add LOS descriptions
        los_descriptions = {
            'A': 'Free flow - Excellent',
            'B': 'Reasonably free flow - Good',
            'C': 'Stable flow - Acceptable',
            'D': 'Approaching unstable - Tolerable',
            'E': 'Unstable flow - Poor',
            'F': 'Forced flow/Breakdown - Very Poor'
        }
        if los != 'N/A':
            desc_text = f"Description: {los_descriptions.get(los, 'Unknown')}"
            ax_los.text(0.5, -0.15, desc_text, transform=ax_los.transAxes,
                       ha='center', fontsize=10, style='italic')
        
        # 2. Key Performance Metrics (averages)
        ax_metrics = fig.add_subplot(gs[0, 0])
        metrics_labels = ['Avg Travel\nTime (s)', 'Avg Delay\n(s)', 'Avg Speed\n(mph)', 'Free-Flow\n(mph)']
        metrics_values = [avg_travel_time, avg_delay, avg_speed, free_flow_speed]
        colors_metrics = ['#1f77b4', '#ff7f0e', '#2ca02c', '#9467bd']
        bars = ax_metrics.bar(metrics_labels, metrics_values, color=colors_metrics, alpha=0.7)
        ax_metrics.set_ylabel('Value', fontsize=11)
        ax_metrics.set_title('Key Performance Metrics (Averages)', fontsize=12, fontweight='bold')
        ax_metrics.grid(axis='y', alpha=0.3)
        
        # Add value labels
        for bar, val in zip(bars, metrics_values):
            height = bar.get_height()
            ax_metrics.text(bar.get_x() + bar.get_width()/2., height,
                           f'{val:.1f}', ha='center', va='bottom', fontsize=9)
        
        # 3. Speed Ratio and Efficiency
        ax_ratio = fig.add_subplot(gs[0, 2])
        ratio_labels = ['Speed\nRatio', 'Route\nEfficiency']
        ratio_values = [speed_ratio * 100, route_efficiency * 100]
        colors_ratio = ['#d62728', '#2ca02c']
        bars = ax_ratio.bar(ratio_labels, ratio_values, color=colors_ratio, alpha=0.7)
        ax_ratio.set_ylabel('Percentage (%)', fontsize=11)
        ax_ratio.set_title('Speed Ratio & Route Efficiency', fontsize=12, fontweight='bold')
        # Set y-axis limit with padding to prevent overlap with title
        max_value = max(ratio_values) if ratio_values else 100
        y_max = max(110, max_value * 1.15)  # Add 15% padding, minimum 110
        ax_ratio.set_ylim(0, y_max)
        ax_ratio.grid(axis='y', alpha=0.3)
        
        # Add value labels with better positioning
        for bar, val in zip(bars, ratio_values):
            height = bar.get_height()
            # Position label above bar, but adjust if too close to top
            label_y = height + (y_max - height) * 0.02  # 2% of remaining space above bar
            ax_ratio.text(bar.get_x() + bar.get_width()/2., label_y,
                         f'{val:.1f}%', ha='center', va='bottom', fontsize=9)
        
        # 4. Delay Analysis (averages)
        ax_delay = fig.add_subplot(gs[1, 0])
        delay_categories = ['Avg\nDelay', 'Delay per\nVehicle', 'Avg Crossing\nDelay']
        delay_values = [
            avg_delay,
            delay_per_vehicle,
            stats.get('avg_crossing_delay_all', 0)
        ]
        bars = ax_delay.bar(delay_categories, delay_values, color='#ff6347', alpha=0.7)
        ax_delay.set_ylabel('Delay (seconds)', fontsize=11)
        ax_delay.set_title('Delay Analysis', fontsize=12, fontweight='bold')
        ax_delay.grid(axis='y', alpha=0.3)
        
        # Add value labels
        for bar, val in zip(bars, delay_values):
            height = bar.get_height()
            ax_delay.text(bar.get_x() + bar.get_width()/2., height,
                         f'{val:.1f}', ha='center', va='bottom', fontsize=9)
        
        # 5. Vehicle Statistics
        ax_vehicles = fig.add_subplot(gs[1, 2])
        vehicle_labels = ['Total\nVehicles', 'Crossing\nAffected']
        vehicle_values = [
            stats.get('total_vehicles_seen', 0),
            stats.get('vehicles_affected_by_crossings', 0)
        ]
        bars = ax_vehicles.bar(vehicle_labels, vehicle_values, color='#1f77b4', alpha=0.7)
        ax_vehicles.set_ylabel('Number of Vehicles', fontsize=11)
        ax_vehicles.set_title('Vehicle Statistics', fontsize=12, fontweight='bold')
        ax_vehicles.grid(axis='y', alpha=0.3)
        
        # Set y-axis limits with padding to prevent label overlap
        y_max = max(vehicle_values) if vehicle_values else 1
        ax_vehicles.set_ylim(0, y_max * 1.2)  # 20% padding for labels
        
        # Add value labels with better positioning
        for bar, val in zip(bars, vehicle_values):
            height = bar.get_height()
            # Position label above bar with padding
            label_y = height + (y_max * 1.2 - height) * 0.05  # 5% above bar
            ax_vehicles.text(bar.get_x() + bar.get_width()/2., label_y,
                            f'{int(val)}', ha='center', va='bottom', fontsize=10,
                            fontweight='bold', color='#1f77b4')
        
        # 6. Crossing Impact (if available)
        ax_crossing = fig.add_subplot(gs[2, :])
        crossing_stats = stats.get('crossing_stats', {})
        if crossing_stats:
            crossing_ids = list(crossing_stats.keys())
            vehicles_affected = [crossing_stats[cid].get('vehicles_affected', 0) 
                                for cid in crossing_ids]
            avg_delays = [crossing_stats[cid].get('avg_delay_per_vehicle', 0) 
                         for cid in crossing_ids]
            durations = [crossing_stats[cid].get('duration', 0) 
                         for cid in crossing_ids]
            
            x = np.arange(len(crossing_ids))
            width = 0.22  # Slightly narrower bars for better spacing
            
            # Create bars for vehicles affected
            bars1 = ax_crossing.bar(x - width, vehicles_affected, width, 
                           label='Vehicles Affected', color='#1f77b4', alpha=0.7)
            ax_crossing2 = ax_crossing.twinx()
            bars2 = ax_crossing2.bar(x, avg_delays, width,
                            label='Avg Delay (s)', color='#ff6347', alpha=0.7)
            bars3 = ax_crossing2.bar(x + width, durations, width,
                            label='Duration (s)', color='#2ca02c', alpha=0.7)
            
            ax_crossing.set_xlabel('Train Crossing', fontsize=11)
            ax_crossing.set_ylabel('Vehicles Affected (Per Crossing)', fontsize=11, color='#1f77b4')
            ax_crossing2.set_ylabel('Time (seconds)', fontsize=11, color='#ff6347')
            ax_crossing.set_title('Train Crossing Impact Analysis', fontsize=12, fontweight='bold')
            ax_crossing.set_xticks(x)
            ax_crossing.set_xticklabels(crossing_ids, rotation=15, ha='right')
            ax_crossing.grid(axis='y', alpha=0.3)
            
            # Get y-axis limits to position labels properly
            y_max_left = max(vehicles_affected) if vehicles_affected else 1
            y_max_right = max(max(avg_delays) if avg_delays else 0, 
                            max(durations) if durations else 0)
            
            # Set y-axis limits with padding to prevent label overlap
            ax_crossing.set_ylim(0, y_max_left * 1.25)  # 25% padding for labels
            ax_crossing2.set_ylim(0, y_max_right * 1.25)
            
            # Add value labels on bars with better positioning to avoid overlap
            for bar, val in zip(bars1, vehicles_affected):
                if val > 0:  # Only show label if value > 0
                    height = bar.get_height()
                    # Position label above bar with padding
                    label_y = height + (y_max_left * 1.25 - height) * 0.05  # 5% above bar
                    ax_crossing.text(bar.get_x() + bar.get_width()/2., label_y,
                                   f'{int(val)}', ha='center', va='bottom', fontsize=9, 
                                   fontweight='bold', color='#1f77b4')
            
            for bar, val in zip(bars2, avg_delays):
                if val > 0:
                    height = bar.get_height()
                    # Position label above bar with padding
                    label_y = height + (y_max_right * 1.25 - height) * 0.05
                    ax_crossing2.text(bar.get_x() + bar.get_width()/2., label_y,
                                    f'{val:.1f}', ha='center', va='bottom', fontsize=9,
                                    fontweight='bold', color='#ff6347')
            
            for bar, val in zip(bars3, durations):
                if val > 0:
                    height = bar.get_height()
                    # Position label above bar with padding
                    label_y = height + (y_max_right * 1.25 - height) * 0.05
                    ax_crossing2.text(bar.get_x() + bar.get_width()/2., label_y,
                                    f'{int(val)}', ha='center', va='bottom', fontsize=9,
                                    fontweight='bold', color='#2ca02c')
            
            # Add legends
            ax_crossing.legend(loc='upper left')
            ax_crossing2.legend(loc='upper right')
            
            # Add note if no vehicles affected
            if sum(vehicles_affected) == 0:
                ax_crossing.text(0.5, 0.95, 
                               'Note: No vehicles were detected as affected by train crossings.\n'
                               'This may indicate trains crossed when no vehicles were present,\n'
                               'or vehicles were not on routes intersecting crossing areas.',
                               transform=ax_crossing.transAxes,
                               ha='center', va='top', fontsize=9, 
                               style='italic', bbox=dict(boxstyle='round', 
                               facecolor='wheat', alpha=0.5))
        else:
            ax_crossing.text(0.5, 0.5, 'No crossing statistics available', 
                           ha='center', va='center', transform=ax_crossing.transAxes,
                           fontsize=12, style='italic')
            ax_crossing.set_title('Train Crossing Impact', fontsize=12, fontweight='bold')
            ax_crossing.axis('off')
        
        plt.tight_layout()
        output_path = os.path.join(self.output_dir, filename)
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"[OK] Saved baseline metrics plot: {output_path}")
    
    def create_all_plots(self, baseline_stats: Optional[Dict] = None, 
                        rerouting_stats: Optional[Dict] = None):
        """Create all visualization plots.
        
        Args:
            baseline_stats: Baseline statistics (optional)
            rerouting_stats: Rerouting statistics (optional)
        """
        if baseline_stats and rerouting_stats:
            self.plot_comparison(baseline_stats, rerouting_stats)
        
        if baseline_stats:
            self.plot_baseline_metrics(baseline_stats, "baseline_metrics.png")
        
        if rerouting_stats:
            crossing_stats = rerouting_stats.get('crossing_stats', {})
            if crossing_stats:
                self.plot_crossing_impact(crossing_stats)
            self.plot_rerouting_statistics(rerouting_stats)
        
        print(f"\n[OK] All plots saved to: {self.output_dir}/")

