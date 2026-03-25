import os
from datetime import datetime
import pandas as pd
import numpy as np
from scipy.stats import ttest_rel, t
import matplotlib.pyplot as plt

# Timestamped output folder so each run writes to new files (avoids permission denied when CSVs are open in Excel)
OUT_DIR = os.path.join("outputs", datetime.now().strftime("run_%Y-%m-%d_%H-%M-%S"))
os.makedirs(OUT_DIR, exist_ok=True)
print(f"Output folder: {OUT_DIR}")

# ==========================================================
# LOAD DATA
# ==========================================================

file = "unified_analysis_20seeds.csv"
df = pd.read_csv(file)

print("\nLoaded results:")
print(df.head())

print("\nColumns in dataset:")
print(df.columns.tolist())

# ==========================================================
# COLUMN DEFINITIONS
# ==========================================================

SEED_COL = "seed"
SCENARIO_COL = "scenario"

DELAY_COL = "average_delay"
CROSSING_DELAY_COL = "crossing_delay"
QUEUE_COL = "queue_length"
LOS_COL = "los"
SPEED_RATIO_COL = "speed_ratio_efficiency"
AFFECTED_COL = "vehicles_affected"
TOTAL_DELAY_COL = "total_delay"

# ==========================================================
# CLEAN SCENARIO NAMES
# ==========================================================

df[SCENARIO_COL] = df[SCENARIO_COL].astype(str).str.strip().str.lower()

baseline = df[df[SCENARIO_COL] == "baseline"]
naive = df[df[SCENARIO_COL] == "naive"]
rerouting = df[df[SCENARIO_COL] == "rerouting"]

# ==========================================================
# ENSURE PAIRED SEEDS
# ==========================================================

common_seeds = sorted(set(baseline[SEED_COL]) &
                      set(naive[SEED_COL]) &
                      set(rerouting[SEED_COL]))

baseline = baseline[baseline[SEED_COL].isin(common_seeds)].sort_values(SEED_COL)
naive = naive[naive[SEED_COL].isin(common_seeds)].sort_values(SEED_COL)
rerouting = rerouting[rerouting[SEED_COL].isin(common_seeds)].sort_values(SEED_COL)

print("\nNumber of paired seeds:", len(common_seeds))

# ==========================================================
# HELPER FUNCTIONS
# ==========================================================

def descriptive_stats(series):
    return {
        "mean": np.mean(series),
        "std": np.std(series, ddof=1),
        "min": np.min(series),
        "max": np.max(series),
        "n": len(series)
    }


def paired_test(x, y, metric, label_x, label_y):
    # x, y must be same-length arrays (.values) so diff is element-wise; n = number of pairs
    diff = x - y
    n = len(diff)  # number of pairs (e.g. 20 paired seeds -> n = 20)

    mean_diff = np.mean(diff)
    std_diff = np.std(diff, ddof=1)

    t_stat, p_two = ttest_rel(x, y)

    # Confidence interval
    se = std_diff / np.sqrt(n)
    tcrit = t.ppf(0.975, df=n-1)

    ci_low = mean_diff - tcrit * se
    ci_high = mean_diff + tcrit * se

    # Cohen's d
    cohens_d = mean_diff / std_diff if std_diff != 0 else np.nan

    # Use Python floats so CSV/Excel export never loses values (no numpy scalar issues)
    return {
        "Metric": metric,
        "Comparison": f"{label_x} vs {label_y}",
        "Mean_X": float(np.mean(x)),
        "Mean_Y": float(np.mean(y)),
        "Mean_Diff": float(mean_diff),
        "Std_Diff": float(std_diff),
        "t_stat": float(t_stat),
        "p_value": float(p_two),
        "CI_low": float(ci_low),
        "CI_high": float(ci_high),
        "Cohen_d": float(cohens_d) if np.isfinite(cohens_d) else np.nan,
        "n": int(n),
    }

# ==========================================================
# DESCRIPTIVE STATISTICS
# ==========================================================

metrics = {
    "Average Delay": DELAY_COL,
    "Crossing Delay": CROSSING_DELAY_COL,
    "Queue Length": QUEUE_COL,
    "Speed Ratio Efficiency": SPEED_RATIO_COL,
    "Vehicles Affected": AFFECTED_COL,
    "Total Delay": TOTAL_DELAY_COL
}

desc_rows = []

for metric_name, col in metrics.items():

    if col not in df.columns:
        continue

    for scenario, data in [
        ("Baseline", baseline),
        ("Naive", naive),
        ("Rerouting", rerouting)
    ]:

        stats = descriptive_stats(data[col])

        desc_rows.append({
            "Metric": metric_name,
            "Scenario": scenario,
            "Mean": stats["mean"],
            "Std": stats["std"],
            "Min": stats["min"],
            "Max": stats["max"],
            "n": stats["n"]
        })

desc_table = pd.DataFrame(desc_rows)

desc_table.to_csv(os.path.join(OUT_DIR, "table_descriptive_statistics.csv"), index=False)

print("\nSaved descriptive statistics table")

# ==========================================================
# PAIRED T-TESTS
# ==========================================================

results = []

# Use .values so paired comparisons are element-wise (same row order); otherwise Series align by index and produce NaN
# Average Delay
results.append(
    paired_test(
        baseline[DELAY_COL].values, rerouting[DELAY_COL].values,
        "Average Delay", "Baseline", "Rerouting"
    )
)

results.append(
    paired_test(
        naive[DELAY_COL].values, rerouting[DELAY_COL].values,
        "Average Delay", "Naive", "Rerouting"
    )
)

# Crossing delay
results.append(
    paired_test(
        baseline[CROSSING_DELAY_COL].values, rerouting[CROSSING_DELAY_COL].values,
        "Crossing Delay", "Baseline", "Rerouting"
    )
)

results.append(
    paired_test(
        naive[CROSSING_DELAY_COL].values, rerouting[CROSSING_DELAY_COL].values,
        "Crossing Delay", "Naive", "Rerouting"
    )
)

# Queue length
results.append(
    paired_test(
        baseline[QUEUE_COL].values, rerouting[QUEUE_COL].values,
        "Queue Length", "Baseline", "Rerouting"
    )
)

results.append(
    paired_test(
        naive[QUEUE_COL].values, rerouting[QUEUE_COL].values,
        "Queue Length", "Naive", "Rerouting"
    )
)

# Speed ratio
results.append(
    paired_test(
        rerouting[SPEED_RATIO_COL].values, baseline[SPEED_RATIO_COL].values,
        "Speed Efficiency", "Rerouting", "Baseline"
    )
)

results.append(
    paired_test(
        rerouting[SPEED_RATIO_COL].values, naive[SPEED_RATIO_COL].values,
        "Speed Efficiency", "Rerouting", "Naive"
    )
)

# Vehicles affected
results.append(
    paired_test(
        baseline[AFFECTED_COL].values, rerouting[AFFECTED_COL].values,
        "Vehicles Affected", "Baseline", "Rerouting"
    )
)

results.append(
    paired_test(
        naive[AFFECTED_COL].values, rerouting[AFFECTED_COL].values,
        "Vehicles Affected", "Naive", "Rerouting"
    )
)

# Total delay
results.append(
    paired_test(
        baseline[TOTAL_DELAY_COL].values, rerouting[TOTAL_DELAY_COL].values,
        "Total Delay", "Baseline", "Rerouting"
    )
)

results.append(
    paired_test(
        naive[TOTAL_DELAY_COL].values, rerouting[TOTAL_DELAY_COL].values,
        "Total Delay", "Naive", "Rerouting"
    )
)

def to_3dec(x):
    # Convert numpy scalars to Python float so we don't miss them
    if hasattr(x, "item"):
        try:
            x = x.item()
        except (ValueError, AttributeError):
            pass
    if x is None or (isinstance(x, float) and (pd.isna(x) or not np.isfinite(x))):
        return ""
    try:
        v = float(x)
        return f"{round(v, 3):.3f}"
    except (TypeError, ValueError):
        return "" if pd.isna(x) else str(x)

# Build output rows: format each numeric field explicitly so no column is lost
def format_result_row(row):
    pv = row["p_value"]
    if hasattr(pv, "item"):
        pv = pv.item()
    p_str = "<0.001" if isinstance(pv, (int, float)) and pv < 0.001 else to_3dec(pv)
    return {
        "Metric": row["Metric"],
        "Comparison": row["Comparison"],
        "Mean_X": to_3dec(row["Mean_X"]),
        "Mean_Y": to_3dec(row["Mean_Y"]),
        "Mean_Diff": to_3dec(row["Mean_Diff"]),
        "Std_Diff": to_3dec(row["Std_Diff"]),
        "t_stat": to_3dec(row["t_stat"]),
        "p_value": p_str,
        "CI_low": to_3dec(row["CI_low"]),
        "CI_high": to_3dec(row["CI_high"]),
        "Cohen_d": to_3dec(row["Cohen_d"]),
        "n": to_3dec(row["n"]),
    }

out_rows = [format_result_row(row) for row in results]
results_df = pd.DataFrame(out_rows)

# Write CSV with all fields quoted so Excel doesn't drop columns when opening
results_df.to_csv(
    os.path.join(OUT_DIR, "table_ttest_results.csv"),
    index=False,
    quoting=1,
)
# Also write Excel file so every column displays correctly (no CSV import issues)
try:
    results_df.to_excel(
        os.path.join(OUT_DIR, "table_ttest_results.xlsx"),
        index=False,
    )
    print("\nSaved t-test results table (CSV and XLSX)")
except Exception as e:
    print("\nSaved t-test results table (CSV only; XLSX skipped:", e, ")")

# ==========================================================
# IMPROVEMENT PERCENTAGES
# ==========================================================

improvements = []

for metric_name, col in metrics.items():

    if col not in df.columns:
        continue

    base = baseline[col].mean()
    reroute = rerouting[col].mean()

    improvement = ((base - reroute) / base) * 100

    improvements.append({
        "Metric": metric_name,
        "Baseline_Mean": base,
        "Rerouting_Mean": reroute,
        "Improvement_%": improvement
    })

improve_df = pd.DataFrame(improvements)

improve_df.to_csv(os.path.join(OUT_DIR, "table_improvements.csv"), index=False)

print("\nSaved improvement table")

# ==========================================================
# LOS SUMMARY
# ==========================================================

los_summary = df.groupby(SCENARIO_COL)[LOS_COL].value_counts().unstack(fill_value=0)

los_summary.to_csv(os.path.join(OUT_DIR, "table_los_summary.csv"))

print("\nSaved LOS summary")

# ==========================================================
# PLOTS
# ==========================================================

# Boxplot
plt.figure(figsize=(8,5))

plt.boxplot(
    [
        baseline[DELAY_COL],
        naive[DELAY_COL],
        rerouting[DELAY_COL]
    ],
    tick_labels=["Baseline","Naive","Rerouting"]
)

plt.ylabel("Average Delay (s)")
plt.title("Average Delay Distribution by Scenario")

plt.grid(True, linestyle="--", alpha=0.5)

plt.savefig(os.path.join(OUT_DIR, "figure_delay_boxplot.png"), dpi=300)
plt.show()

# Bar chart with std

means = [
    baseline[DELAY_COL].mean(),
    naive[DELAY_COL].mean(),
    rerouting[DELAY_COL].mean()
]

stds = [
    baseline[DELAY_COL].std(),
    naive[DELAY_COL].std(),
    rerouting[DELAY_COL].std()
]

plt.figure(figsize=(8,5))

plt.bar(
    ["Baseline","Naive","Rerouting"],
    means,
    yerr=stds,
    capsize=5
)

plt.ylabel("Average Delay (s)")
plt.title("Average Delay with Standard Deviation")

plt.grid(True, linestyle="--", alpha=0.5)

plt.savefig(os.path.join(OUT_DIR, "figure_delay_barplot.png"), dpi=300)
plt.show()

print("\nPlots saved")

print(f"\nAll outputs saved to folder: {OUT_DIR}")
print("Analysis complete.")