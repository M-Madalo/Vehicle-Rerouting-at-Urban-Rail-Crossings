import pandas as pd
import numpy as np
from scipy.stats import ttest_rel, t
import matplotlib.pyplot as plt

# ==============================
# Load simulation results
# ==============================
file = "unified_analysis_20seeds.csv"
df = pd.read_csv(file)

print("\nLoaded results:")
print(df.head())
print("\nColumns in CSV:")
print(df.columns.tolist())

# ==============================
# Column mapping
# ==============================
# Adjust these only if your CSV uses different names
SEED_COL = "seed"
SCENARIO_COL = "scenario"
DELAY_COL = "average_delay"
CROSSING_DELAY_COL = "crossing_delay"
QUEUE_COL = "queue_length"
LOS_COL = "los"
SPEED_RATIO_COL = "speed_ratio_efficiency"
AFFECTED_COL = "vehicles_affected"
TOTAL_DELAY_COL = "total_delay"

# ==============================
# Clean scenario names
# ==============================
df[SCENARIO_COL] = df[SCENARIO_COL].astype(str).str.strip().str.lower()

# ==============================
# Split scenarios
# ==============================
baseline = df[df[SCENARIO_COL] == "baseline"].sort_values(SEED_COL).reset_index(drop=True)
naive = df[df[SCENARIO_COL] == "naive"].sort_values(SEED_COL).reset_index(drop=True)
rerouting = df[df[SCENARIO_COL] == "rerouting"].sort_values(SEED_COL).reset_index(drop=True)

print("\nScenario counts:")
print("Baseline:", len(baseline))
print("Naive:", len(naive))
print("Rerouting:", len(rerouting))

# Check paired seeds
common_seeds = sorted(set(baseline[SEED_COL]) & set(naive[SEED_COL]) & set(rerouting[SEED_COL]))
baseline = baseline[baseline[SEED_COL].isin(common_seeds)].sort_values(SEED_COL).reset_index(drop=True)
naive = naive[naive[SEED_COL].isin(common_seeds)].sort_values(SEED_COL).reset_index(drop=True)
rerouting = rerouting[rerouting[SEED_COL].isin(common_seeds)].sort_values(SEED_COL).reset_index(drop=True)

print("\nCommon paired seeds:", len(common_seeds))
print(common_seeds)

# ==============================
# Helper functions
# ==============================
def descriptive_stats(series):
    return {
        "mean": np.mean(series),
        "std": np.std(series, ddof=1),
        "min": np.min(series),
        "max": np.max(series),
        "n": len(series)
    }

def paired_test(x, y, metric_name, label_x, label_y):
    """
    Tests whether x > y on average.
    Example: baseline vs rerouting for delay.
    """
    diff = x - y
    n = len(diff)
    mean_diff = np.mean(diff)
    std_diff = np.std(diff, ddof=1)
    se = std_diff / np.sqrt(n)

    t_stat, p_two_tailed = ttest_rel(x, y)
    p_one_tailed = p_two_tailed / 2 if t_stat > 0 else 1 - (p_two_tailed / 2)

    # 95% CI for paired mean difference
    t_crit = t.ppf(0.975, df=n - 1)
    ci_low = mean_diff - t_crit * se
    ci_high = mean_diff + t_crit * se

    # Cohen's d for paired samples
    cohens_d = mean_diff / std_diff if std_diff != 0 else np.nan

    print("\n" + "=" * 60)
    print(f"PAIRED T-TEST: {metric_name}")
    print(f"{label_x} vs {label_y}")
    print("=" * 60)
    print(f"Mean {label_x}: {np.mean(x):.4f}")
    print(f"Mean {label_y}: {np.mean(y):.4f}")
    print(f"Mean difference ({label_x} - {label_y}): {mean_diff:.4f}")
    print(f"Std of differences: {std_diff:.4f}")
    print(f"t-statistic: {t_stat:.4f}")
    print(f"Two-tailed p-value: {p_two_tailed:.8f}")
    print(f"One-tailed p-value: {p_one_tailed:.8f}")
    print(f"95% CI of difference: [{ci_low:.4f}, {ci_high:.4f}]")
    print(f"Cohen's d (paired): {cohens_d:.4f}")

    return {
        "metric": metric_name,
        "comparison": f"{label_x} vs {label_y}",
        "mean_x": np.mean(x),
        "mean_y": np.mean(y),
        "mean_diff": mean_diff,
        "std_diff": std_diff,
        "t_stat": t_stat,
        "p_two_tailed": p_two_tailed,
        "p_one_tailed": p_one_tailed,
        "ci_low": ci_low,
        "ci_high": ci_high,
        "cohens_d": cohens_d,
        "n": n
    }

# ==============================
# Descriptive statistics
# ==============================
metrics = {
    "Average Delay": DELAY_COL,
    "Crossing Delay": CROSSING_DELAY_COL,
    "Queue Length": QUEUE_COL,
    "Speed Ratio Efficiency": SPEED_RATIO_COL,
    "Vehicles Affected": AFFECTED_COL,
    "Total Delay": TOTAL_DELAY_COL
}

print("\n" + "=" * 60)
print("DESCRIPTIVE STATISTICS")
print("=" * 60)

for metric_name, col in metrics.items():
    if col not in df.columns:
        print(f"\nSkipping {metric_name}: column '{col}' not found.")
        continue

    print(f"\n--- {metric_name} ---")
    for scenario_name, scenario_df in [("Baseline", baseline), ("Naive", naive), ("Rerouting", rerouting)]:
        stats = descriptive_stats(scenario_df[col].values)
        print(
            f"{scenario_name}: "
            f"mean={stats['mean']:.4f}, std={stats['std']:.4f}, "
            f"min={stats['min']:.4f}, max={stats['max']:.4f}, n={stats['n']}"
        )

# ==============================
# Paired t-tests
# ==============================
results = []

# Delay
results.append(
    paired_test(
        baseline[DELAY_COL].values,
        rerouting[DELAY_COL].values,
        "Average Delay",
        "Baseline",
        "Rerouting"
    )
)

results.append(
    paired_test(
        naive[DELAY_COL].values,
        rerouting[DELAY_COL].values,
        "Average Delay",
        "Naive",
        "Rerouting"
    )
)

# Crossing delay
if CROSSING_DELAY_COL in df.columns:
    results.append(
        paired_test(
            baseline[CROSSING_DELAY_COL].values,
            rerouting[CROSSING_DELAY_COL].values,
            "Crossing Delay",
            "Baseline",
            "Rerouting"
        )
    )
    results.append(
        paired_test(
            naive[CROSSING_DELAY_COL].values,
            rerouting[CROSSING_DELAY_COL].values,
            "Crossing Delay",
            "Naive",
            "Rerouting"
        )
    )

# Queue length
if QUEUE_COL in df.columns:
    results.append(
        paired_test(
            baseline[QUEUE_COL].values,
            rerouting[QUEUE_COL].values,
            "Queue Length",
            "Baseline",
            "Rerouting"
        )
    )
    results.append(
        paired_test(
            naive[QUEUE_COL].values,
            rerouting[QUEUE_COL].values,
            "Queue Length",
            "Naive",
            "Rerouting"
        )
    )

# Speed ratio: here higher is better, so compare rerouting - baseline
if SPEED_RATIO_COL in df.columns:
    results.append(
        paired_test(
            rerouting[SPEED_RATIO_COL].values,
            baseline[SPEED_RATIO_COL].values,
            "Speed Ratio Efficiency",
            "Rerouting",
            "Baseline"
        )
    )
    results.append(
        paired_test(
            rerouting[SPEED_RATIO_COL].values,
            naive[SPEED_RATIO_COL].values,
            "Speed Ratio Efficiency",
            "Rerouting",
            "Naive"
        )
    )

# Vehicles affected: lower is better
if AFFECTED_COL in df.columns:
    results.append(
        paired_test(
            baseline[AFFECTED_COL].values,
            rerouting[AFFECTED_COL].values,
            "Vehicles Affected",
            "Baseline",
            "Rerouting"
        )
    )
    results.append(
        paired_test(
            naive[AFFECTED_COL].values,
            rerouting[AFFECTED_COL].values,
            "Vehicles Affected",
            "Naive",
            "Rerouting"
        )
    )

# Total delay: lower is better
if TOTAL_DELAY_COL in df.columns:
    results.append(
        paired_test(
            baseline[TOTAL_DELAY_COL].values,
            rerouting[TOTAL_DELAY_COL].values,
            "Total Delay",
            "Baseline",
            "Rerouting"
        )
    )
    results.append(
        paired_test(
            naive[TOTAL_DELAY_COL].values,
            rerouting[TOTAL_DELAY_COL].values,
            "Total Delay",
            "Naive",
            "Rerouting"
        )
    )

# ==============================
# Save t-test summary
# ==============================
results_df = pd.DataFrame(results)
results_df.to_csv("paired_ttest_results.csv", index=False)

print("\nSaved paired t-test results to paired_ttest_results.csv")

# ==============================
# LOS summary
# ==============================
if LOS_COL in df.columns:
    print("\n" + "=" * 60)
    print("LOS SUMMARY")
    print("=" * 60)
    los_summary = df.groupby(SCENARIO_COL)[LOS_COL].value_counts().unstack(fill_value=0)
    print(los_summary)
    los_summary.to_csv("los_summary.csv")

# ==============================
# Plot average delay by scenario
# ==============================
plt.figure(figsize=(8, 5))
plt.boxplot(
    [
        baseline[DELAY_COL].values,
        naive[DELAY_COL].values,
        rerouting[DELAY_COL].values
    ],
    tick_labels=["Baseline", "Naive", "Rerouting"]
)
plt.ylabel("Average Delay (s)")
plt.title("Paired Simulation Results: Average Delay by Scenario")
plt.grid(True, linestyle="--", alpha=0.5)
plt.tight_layout()
plt.savefig("average_delay_boxplot.png", dpi=300)
plt.show()

print("\nSaved plot: average_delay_boxplot.png")
print("\nAnalysis complete.")