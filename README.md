# Documentation: Vehicle Rerouting at Urban Rail Crossings

This document describes the  design of **`enhanced_dijkstra.py`** and **`sumo_controller.py`** which they are the main file. Together they implement **grade-crossing-aware vehicle rerouting** in SUMO simulations: when a train is detected at a crossing, road edges at the crossing are blocked and affected vehicles are rerouted around them using a custom Dijkstra-based router.

---

## 1. Overview

- **Goal:** In a SUMO simulation that includes both road traffic and trains (e.g., 4th Avenue scenario), detect when a train is at a grade crossing, block the affected road edges, and **reroute** road vehicles around the blockage using an **Enhanced Dijkstra** router instead of letting them queue at the crossing.
- **Main components:**
  - **`enhanced_dijkstra.py`** – Builds an edge-to-edge graph from the SUMO network and provides shortest-path and alternative-path computation with support for blocked edges, congestion, and train crossings.
  - **`sumo_controller.py`** – Drives the SUMO simulation via TraCI, detects trains and active crossings, maintains the routing graph at runtime, and calls the Enhanced Dijkstra to compute and apply reroutes.
- **Entry point for analysis:** **`run_rerouting_analysis.py`** – Initializes `EnhancedDijkstra` from the net.xml, creates a `SUMOController` with rerouting enabled, runs the simulation, collects metrics (delay, LOS, rerouting stats), and can generate visualizations.

---

## 2. Enhanced Dijkstra (`enhanced_dijkstra.py`)

### 2.1 Purpose

- Build a **deterministic edge-to-edge graph** from a SUMO `.net.xml` using `<edge>` and `<connection>` elements so that only **legal vehicle movements** are represented.
- Provide a **drop-in compatible** API for the controller: path calculation, edge blocking, congestion updates, and train-crossing registration.

### 2.2 Data Structures

- **`Edge` (dataclass)**  
  Represents one graph arc: `from_node`, `to_node`, `edge_id`, `length`, `max_speed`, `base_weight`, `current_weight`, and optional `is_blocked`, `congestion_factor`, `vehicle_count`.  
  In this design, **nodes are edge IDs** (edge-to-edge graph): e.g. `from_node` = previous edge ID, `to_node` = current edge ID, `edge_id` = the edge being traversed.

- **`TrainCrossing` (dataclass)**  
  Represents an active crossing: `crossing_id`, `edge_ids` (set of blocked edge IDs), `start_time`, `end_time`, `severity`.

### 2.3 Graph Building from SUMO Network

- **`create_graph_from_sumo_network(network_file)`**  
  Parses the `.net.xml`, uses **`<connection>`** elements to determine legal successor edges, and builds:
  - **`graph`:** `Dict[from_edge_id, List[Edge]]` (adjacency list, edge-to-edge).
  - **`edge_meta`:** `Dict[edge_id, {length, max_speed, base_weight}]` from lane lengths and speeds.
- Internal edges (e.g. `function="internal"` or IDs starting with `:`) are skipped.
- **`EnhancedDijkstra.from_sumo_netxml(network_file, ...)`** – Class method that builds the graph and edge metadata and returns an `EnhancedDijkstra` instance (used in `run_rerouting_analysis.py` with `4thAve.net.xml`).

### 2.4 Core API Used by the Controller

- **`load_from_netxml(network_file)`** – Rebuilds the graph and edge metadata from a net.xml (optional reload).
- **`update_edge_weight(edge_id, new_weight)`** – Sets `current_weight` for an edge (e.g. after congestion).
- **`update_congestion(edge_id, vehicle_count)`** – Updates real-time and historical vehicle counts and applies a simple multiplicative congestion factor to `current_weight`.
- **`block_edge(edge_id, block=True/False)`** – Marks an edge as blocked (weight set to infinity when blocked) or unblocks it.
- **`register_train_crossing(crossing)`** – Registers a `TrainCrossing` and blocks all edges in `crossing.edge_ids`.
- **`remove_train_crossing(crossing_id)`** – Unregisters the crossing and unblocks those edges.
- **`get_affected_edges(crossing_id)`** – Returns the set of edge IDs affected by that crossing.
- **`calculate_path(start_node, end_node, current_time, avoid_edges, priority)`** – Computes shortest path from `start_node` to `end_node` (both are edge IDs in the edge-to-edge graph), optionally avoiding a set of edges (e.g. blocked crossing edges). Returns `(list of edge IDs, cost)`.
- **`find_alternative_path(start_node, end_node, original_path, blocked_edges, current_time, priority)`** – Wrapper that calls `calculate_path` with `avoid_edges=blocked_edges`; used by the controller for rerouting.

### 2.5 Routing Modes (Priority)

- **`priority_mode`** (instance/config): `"time"`, `"distance"`, or `"balanced"`.
- **`balanced_time_weight`** / **`balanced_distance_weight`** – Used when mode is `"balanced"` to combine normalized time and distance into a single edge cost.
- Edge cost can also be adjusted by **predicted congestion** and **edge importance** (e.g. for future extensions).

---

## 3. SUMO Controller (`sumo_controller.py`)

### 3.1 Role

- **Start and drive** the SUMO simulation (e.g. `4thave.sumocfg`) via TraCI.
- **Initialize network data** from the running simulation: build `edge_info` (length, max_speed, outgoing edges) and **populate the same `EnhancedDijkstra` instance’s graph** using TraCI (lane links and vehicle routes). So at runtime the controller may **replace or augment** the graph that was loaded from net.xml with connections discovered from the live simulation.
- **Detect trains** (by type or ID) and **detect active train crossings** (train position + affected road edges).
- **Block only grade-crossing road edges** (optional, see `block_only_grade_crossing_roads`) and known `GRADE_CROSSING_ROADS` so that not every edge touching the train’s junction is blocked.
- **Decide which vehicles need rerouting** (route intersects blocked edges, approaching blocked area, or stopped at crossing).
- **Compute alternative routes** via `dijkstra.find_alternative_path(...)` and apply them with `traci.vehicle.setRoute(...)`, with fallback to `traci.vehicle.rerouteTraveltime(...)` if the custom router fails.
- **Collect metrics**: per-vehicle (travel time, delay, crossing delay, was_rerouted, crossing_affected) and per-crossing (vehicles affected, queue length, total/average delay).

### 3.2 Initialization and Graph Building

- **Constructor** takes `sumo_config`, an **`EnhancedDijkstra`** instance, and many options (e.g. `enable_rerouting`, `rerouting_strategy`, `train_crossing_duration`, `block_only_grade_crossing_roads`, `dijkstra_priority`).
- **`start_simulation()`** calls **`_initialize_network()`**, which:
  - Builds **`edge_info`** for all non-internal edges (length, max_speed) via TraCI.
  - Builds **edge-to-edge connections** from vehicle routes and from **lane links** (`traci.lane.getLinks`).
  - Fills **`self.dijkstra.graph`** and **`self.dijkstra.nodes`** with `Edge` objects and node IDs (edge IDs), so pathfinding uses the same edge-to-edge model as `enhanced_dijkstra.py`.

### 3.3 Train and Crossing Detection

- **`_identify_trains()`** / **`update_train_list()`** – Identify vehicles whose type or ID indicates a train.
- **`detect_train_crossings(current_time)`** – For each active train:
  - Gets current and upcoming train edges.
  - Determines **affected road edges** (e.g. edges sharing junctions with the train, plus known `GRADE_CROSSING_ROADS`).
  - Creates or updates a **`TrainCrossing`** and calls **`self.dijkstra.register_train_crossing(crossing)`** (or updates blocked edges if the crossing already exists).
  - When a crossing **expires** (`current_time > end_time`), unblocks edges and calls **`self.dijkstra.remove_train_crossing(crossing_id)`**.

So **blocking/unblocking in the graph is centralized in the Dijkstra instance**; the controller only registers/updates/removes crossings and the Dijkstra module sets edge weights to infinity or back to normal.

### 3.4 Rerouting Logic

- **`reroute_vehicle(vehicle_id, current_time, blocked_edges)`**:
  - Gets current edge, destination edge, and route.
  - Enforces **max reroutes per vehicle**, **reroute cooldown**, and **max reroute failures**.
  - Estimates **remaining cost on current route** and **expected wait** (from active crossings’ end times).
  - Calls **`self.dijkstra.find_alternative_path(current_edge, destination_edge, route, blocked_edges, current_time, priority=self.dijkstra_priority)`**.
  - **Wait-vs-detour rule:** If the alternative path cost is ≥ ~90% of “wait cost” (remaining + expected wait), reroute is **not** applied (avoids bad detours).
  - On success: **`traci.vehicle.setRoute(vehicle_id, candidate_path)`** and updates reroute counters.
  - On failure: fallback to **`traci.vehicle.rerouteTraveltime(vehicle_id)`**.

- **`run_step(current_time)`** (high level):
  1. **`traci.simulationStep()`**
  2. Optionally **build more graph connections** from vehicle routes if the graph is still sparse (e.g. before first train at t=300).
  3. **`update_traffic_congestion()`** – Updates per-edge vehicle counts and **`self.dijkstra.update_congestion(...)`** so path costs reflect current congestion.
  4. **`detect_train_crossings(current_time)`** – Updates active crossings and blocked edges in the Dijkstra graph.
  5. Build **`blocked_edges`** and **predictive blocked edges** (e.g. when train is within ~40 s of crossing).
  6. **Update vehicle and crossing metrics.**
  7. For each non-train vehicle: if **route intersects blocked edges**, or **approaching blocked area**, or **stopped at crossing**, and rerouting is enabled and limits not exceeded → **`reroute_vehicle(...)`**.
  8. Set **high effort** on blocked edges in SUMO (`traci.edge.setEffort`) so SUMO’s own rerouting also avoids them when used.

### 3.5 Metrics and Statistics

- **`vehicle_metrics`** – Per-vehicle: start_time, original_route_length, actual_distance, travel_time, delay_time, crossing_delay, was_rerouted, crossing_affected, speeds.
- **`crossing_metrics`** – Per crossing: vehicles_affected, max_queue_length, total_delay, duration.
- **`get_statistics()`** – Aggregates congestion and travel-time metrics, reroute counts, and crossing stats for use by **`run_rerouting_analysis.py`** and **`MetricsCollector`** (e.g. LOS, avg delay for rerouted vs non-rerouted).

---

## 4. How the Two Modules Work Together

1. **Analysis script** (`run_rerouting_analysis.py`):
   - Creates **`EnhancedDijkstra.from_sumo_netxml("4thAve.net.xml")`** to get an initial graph from the net file.
   - Creates **`SUMOController(sumo_config, dijkstra, enable_rerouting=True, block_only_grade_crossing_roads=True, train_crossing_duration=60.0)`**.
   - Starts the simulation; **`_initialize_network()`** then (re)builds the controller’s view of the graph and **populates the same `dijkstra` instance** with TraCI-derived edges and connections.

2. **Each step** (`controller.run_step(current_sim_time)`):
   - Congestion is pushed into the Dijkstra instance via **`update_congestion`**.
   - **Train crossings** are detected and **blocked edges** are registered/updated/removed in the **Dijkstra** via **`register_train_crossing`** / **`block_edge`** / **`remove_train_crossing`**.
   - Vehicles that need rerouting get **alternative paths** from **`dijkstra.find_alternative_path(...)`** and new routes are applied in SUMO.

3. **After the run**, **`get_statistics()`** and metrics collectors provide **LOS**, **average delay (rerouted vs non-rerouted)**, and **crossing impact**, which can be visualized (e.g. rerouting metrics, crossing impact, rerouting statistics plots).

---

## 5. Design Choices and Recent Behavior

- **Block only grade-crossing roads** – When `block_only_grade_crossing_roads=True`, only known road edges at grade crossings (and junction-based logic with train edges) are blocked, reducing over-blocking and unnecessary reroutes.
- **Train crossing duration** – Configurable (e.g. 60 s in rerouting analysis) so the crossing is considered “active” for a fixed time after detection; edges are unblocked when the crossing expires.
- **Wait-vs-detour** – Reroute is skipped if the computed detour cost is at least 90% of the estimated “wait” cost (remaining route + expected wait), avoiding overly long detours.
- **Reroute cooldown** (e.g. 30 s) and **max reroutes per vehicle** (e.g. 3) – Limit oscillation and excessive rerouting.
- **Predictive blocking** – Edges at known grade crossings can be marked blocked slightly before the train fully arrives (e.g. when train is within ~40 s) to trigger earlier rerouting.
- **Fallback to SUMO** – If the custom Dijkstra pathfinding fails, **`traci.vehicle.rerouteTraveltime(vehicle_id)`** is used so vehicles still get an updated route.

---

## 6. File Dependency Summary

| File | Role |
|------|------|
| **`enhanced_dijkstra.py`** | Graph from net.xml, path calculation, blocking, congestion, train-crossing registration. |
| **`sumo_controller.py`** | TraCI simulation control, network init from TraCI, train/crossing detection, reroute decisions, application of routes, metrics. |
| **`run_rerouting_analysis.py`** | Creates Dijkstra from `4thAve.net.xml`, creates controller with rerouting on, runs simulation, collects metrics and LOS, generates visuals. |
| **`run_no_train_analysis.py`** | from `4thAve.net.xml`, runs simulation without train, collects metrics and LOS, generates visuals. |
| **`run_baseline_analysis.py`** | from `4thAve.net.xml`, introduces both freight trains with rerouting disabled. Vehicles encountering blocked crossings queue until the train clears, representing current conditions with no advance guidance, runs simulation, collects metrics and LOS, generates visuals. |
| **`run_naive_rerouting_analysis.py`** | from `4thAve.net.xml`,implements a proximity-based rerouting strategy to evaluate whether simple spatial awareness can mitigate crossing delays, collects metrics and LOS, generates visuals. |
| **`run_unified_analysis.py`** | from `4thAve.net.xml`,implements 4 scenarios no trains - baseline with train - naive rerouting - intelligent rerouting  with 20 seeds , collects metrics and LOS, generates visuals. |
| **`metrics_collector.py`** | Gathers and aggregates metrics from the controller. |
| **`visualize_metrics.py`** | Plots (e.g. rerouting metrics, crossing impact, rerouting statistics). |

This documentation reflects the behavior of **`enhanced_dijkstra.py`** and **`sumo_controller.py`** as used in the current rerouting analysis pipeline.
