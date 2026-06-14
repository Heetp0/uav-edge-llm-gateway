#!/usr/bin/env python3
"""
plot_results.py
---------------
Generates benchmark analysis charts from the CSV produced by benchmark_pipeline.py.

Usage:
    ros2 run drone_safety plot_results               
    python3 plot_results.py --out /tmp/plots    
"""

import os
import sys
import argparse

import pandas as pd
import matplotlib.pyplot as plt

# ── Default paths ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV = os.path.expanduser("~/quantization_benchmark_results.csv")
DEFAULT_OUT = SCRIPT_DIR


def parse_boolean_column(series: pd.Series) -> pd.Series:
    """Case-insensitive boolean parsing that defaults to False for bad data/timeouts."""
    return series.astype(str).str.strip().str.lower() == "true"


def generate_plots(csv_path: str, output_dir: str):

    if not os.path.exists(csv_path):
        print(f"[ERROR] Data file not found: {csv_path}")
        print("        Run the benchmark first.")
        sys.exit(1)

    os.makedirs(output_dir, exist_ok=True)

    df = pd.read_csv(csv_path)

    if len(df) == 0:
        print("[ERROR] CSV file is empty.")
        sys.exit(1)

    # ── FIX: Master Command List for Total Failure Tracking ──
    all_cmds = pd.to_numeric(df["Command ID"], errors="coerce").dropna().astype(int).unique()
    all_cmds.sort()
    df["Command ID"] = pd.to_numeric(df["Command ID"], errors="coerce").fillna(-1).astype(int)

    # ── FIX: Evaluate Accuracy against ALL attempts (including timeouts) ──
    total_attempts = len(df)
    
    df["Syntactic Success"] = parse_boolean_column(df["Syntactic Success"])
    df["Semantic Success"]  = parse_boolean_column(df["Semantic Success"])

    syntax_acc   = df["Syntactic Success"].sum() / total_attempts * 100
    semantic_acc = df["Semantic Success"].sum()  / total_attempts * 100

    # ── FIX: Isolate Latency Metrics (Exclude Timeouts/Fails) ──
    df_latency = df[pd.to_numeric(df["Latency (ms)"], errors="coerce") > 0].copy()
    df_latency["Latency (ms)"] = pd.to_numeric(df_latency["Latency (ms)"])
    
    timeout_count = total_attempts - len(df_latency)

    def save(filename: str):
        path = os.path.join(output_dir, filename)
        plt.savefig(path, dpi=300)
        plt.close()
        print(f"[SUCCESS] {path}")

    # ── PLOT 1: Latency Distribution (box plot) ───────────────────────────────
    plt.figure(figsize=(6, 8))
    if len(df_latency) > 0:
        plt.boxplot(
            df_latency["Latency (ms)"],
            patch_artist=True,
            boxprops=dict(facecolor="lightgrey"),
        )
    plt.title(
        "Edge Inference Latency (qwen2.5:3b — 4-bit Quantized)",
        fontsize=12, fontweight="bold",
    )
    plt.ylabel("Latency (ms)", fontsize=12)
    plt.xticks([1], ["WSL2 Edge Environment"])
    plt.grid(axis="y", linestyle="--", alpha=0.7)
    plt.tight_layout()
    save("latency_distribution.png")

    # ── PLOT 2: System Accuracy (bar chart) ───────────────────────────────────
    plt.figure(figsize=(7, 5))
    bars = plt.bar(
        ["Syntactic JSON Success", "Semantic Spatial Success"],
        [syntax_acc, semantic_acc],
        color=["#4C72B0", "#55A868"],
    )
    plt.title(
        f"LLM Instruction Translation Accuracy\n"
        f"(n={total_attempts} total attempts | {timeout_count} timeouts penalized)",
        fontsize=11, fontweight="bold",
    )
    plt.ylabel("Success Rate (%)", fontsize=12)
    plt.ylim(0, 110)
    for bar in bars:
        yval = bar.get_height()
        plt.text(
            bar.get_x() + bar.get_width() / 2, yval + 2,
            f"{yval:.1f}%", ha="center", va="bottom", fontsize=11,
        )
    plt.tight_layout()
    save("accuracy_metrics.png")

    # ── PLOT 3: Latency time series ───────────────────────────────────────────
    df_sorted = df_latency.sort_values(
        ["Run ID", "Command ID"]
    ).reset_index(drop=True)

    plt.figure(figsize=(10, 4))
    if len(df_sorted) > 0:
        plt.plot(
            df_sorted["Latency (ms)"].values,
            alpha=0.6, linewidth=0.8, label="Request latency",
        )
        plt.axhline(
            df_sorted["Latency (ms)"].mean(),
            color="red", linestyle="--", linewidth=1.2, label="Mean",
        )
    plt.title(
        "Latency Over Time (Sequential Execution Order)",
        fontsize=12, fontweight="bold",
    )
    plt.xlabel("Request Index (sorted by execution order)")
    plt.ylabel("Latency (ms)")
    plt.legend()
    plt.tight_layout()
    save("latency_time_series.png")

    # ── PLOT 4: Per-command semantic accuracy ─────────────────────────────────
    # Uses master 'df' to guarantee timeouts heavily drag down the average
    cmd_acc = (
        df.groupby("Command ID")["Semantic Success"]
        .mean()
        .reindex(all_cmds, fill_value=0.0) 
        * 100
    )

    # Filter out the fallback -1 index if empty/corrupted rows existed
    if -1 in cmd_acc.index:
        cmd_acc = cmd_acc.drop(-1)

    plt.figure(figsize=(10, 4))
    plt.bar(cmd_acc.index.astype(str), cmd_acc.values, color="#4C72B0")
    plt.title(
        "Semantic Accuracy Per Command (Penalized for Timeouts)",
        fontsize=12, fontweight="bold",
    )
    plt.xlabel("Command ID")
    plt.ylabel("Success Rate (%)")
    plt.ylim(0, 110)
    plt.xticks(rotation=45 if len(cmd_acc) > 15 else 0)
    plt.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    save("per_command_accuracy.png")

    # ── Summary ───────────────────────────────────────────────────────────────
    print("")
    print("=" * 50)
    print(f"  Total Attempts: {total_attempts}")
    print(f"  Valid Latency : {len(df_latency)}")
    print(f"  Network TOs   : {timeout_count}")
    print(f"  Syntax Acc.   : {syntax_acc:.1f}%")
    print(f"  Semant Acc.   : {semantic_acc:.1f}%")
    print(f"  Output dir    : {output_dir}")
    print("=" * 50)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot UAV LLM benchmark results.")
    parser.add_argument(
        "--csv", default=DEFAULT_CSV,
        help=f"Path to benchmark CSV (default: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--out", default=DEFAULT_OUT,
        help=f"Output directory for plots (default: {DEFAULT_OUT})",
    )
    args = parser.parse_args()
    generate_plots(args.csv, args.out)
