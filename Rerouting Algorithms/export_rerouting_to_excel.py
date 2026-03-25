import json
import glob
import os
import csv

BASE_DIR = os.path.dirname(__file__)
pattern = os.path.join(BASE_DIR, "stats_rerouting_seed*.json")

rows = []
for path in sorted(glob.glob(pattern)):
    fname = os.path.basename(path)
    # extract seed number from filename: stats_rerouting_seedXXXX.json
    seed = fname.replace("stats_rerouting_seed", "").replace(".json", "")
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    rer = data.get("rerouting_stats", {})
    crossing = data.get("crossing_stats", {})
    c1 = crossing.get("train_crossing_train_1", {})
    c2 = crossing.get("train_crossing_train_2", {})

    rows.append({
        "seed": seed,
        "total_vehicles": rer.get("total_vehicles_seen", data.get("total_vehicles_seen")),
        "los_all": data.get("los_all"),
        "avg_delay_all_s": data.get("avg_delay_time_all"),
        "avg_travel_time_all_s": data.get("avg_travel_time_all"),
        "avg_speed_all_mps": data.get("avg_speed_all"),
        "vehicles_rerouted": rer.get("unique_vehicles_rerouted"),
        "reroute_ops": rer.get("reroute_count"),
        "crossing1_veh_affected": c1.get("vehicles_affected"),
        "crossing1_max_queue": c1.get("max_queue_length"),
        "crossing2_veh_affected": c2.get("vehicles_affected"),
        "crossing2_max_queue": c2.get("max_queue_length"),
    })

out_path = os.path.join(BASE_DIR, "rerouting_results_summary.csv")
fieldnames = list(rows[0].keys()) if rows else []

with open(out_path, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)

print(f"Wrote {len(rows)} rows to {out_path}")