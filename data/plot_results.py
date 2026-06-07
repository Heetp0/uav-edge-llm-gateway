#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import os


def generate_plots():
    csv_path = os.path.expanduser("~/quantization_benchmark_results.csv")
    if not os.path.exists(csv_path):
        print(f"Error: Data file not found at {csv_path}. Run benchmark first.")
        return

    df = pd.read_csv(csv_path)

    # ── Filter out TIMEOUT rows for latency calculations ──────────────────────
    df_valid = df[df["Latency (ms)"] != "TIMEOUT"].copy()
    df_valid["Latency (ms)"] = pd.to_numeric(df_valid["Latency (ms)"])

    # ── Robust boolean mapping ────────────────────────────────────────────────
    # .map() returns NaN for any value not in the dict (e.g. empty strings);
    # those rows are safely excluded from accuracy totals via .sum() ignoring NaN.
    df_valid["Syntactic Success"] = (
        df_valid["Syntactic Success"].astype(str).str.strip()
        .map({"True": True, "False": False})
    )
    df_valid["Semantic Success"] = (
        df_valid["Semantic Success"].astype(str).str.strip()
        .map({"True": True, "False": False})
    )

    total_runs    = len(df_valid)
    timeout_count = len(df) - total_runs

    syntax_acc   = df_valid["Syntactic Success"].sum() / total_runs * 100 if total_runs else 0
    semantic_acc = df_valid["Semantic Success"].sum()  / total_runs * 100 if total_runs else 0

    # ── PLOT 1: Latency Distribution (box plot) ───────────────────────────────
    plt.figure(figsize=(6, 8))
    plt.boxplot(
        df_valid["Latency (ms)"],
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
    plt.savefig("latency_distribution.png", dpi=300)
    plt.close()
    print("[SUCCESS] Generated latency_distribution.png")

    # ── PLOT 2: System Accuracy (bar chart) ───────────────────────────────────
    plt.figure(figsize=(7, 5))
    bars = plt.bar(
        ["Syntactic JSON Success", "Semantic Spatial Success"],
        [syntax_acc, semantic_acc],
        color=["#4C72B0", "#55A868"],
    )
    plt.title(
        f"LLM Instruction Translation Accuracy\n"
        f"(n={total_runs} valid, {timeout_count} timeouts excluded)",
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
    plt.savefig("accuracy_metrics.png", dpi=300)
    plt.close()
    print("[SUCCESS] Generated accuracy_metrics.png")

    # ── PLOT 3: Latency time series ───────────────────────────────────────────
    # Sort by (Run ID, Command ID) to reconstruct true execution order.
    # Essential to prevent resumed benchmarks from appearing out of sequence.
    df_sorted = df_valid.sort_values(
        ["Run ID", "Command ID"]
    ).reset_index(drop=True)

    plt.figure(figsize=(10, 4))
    plt.plot(df_sorted["Latency (ms)"].values, alpha=0.6, linewidth=0.8,
             label="Request latency")
    plt.axhline(
        df_sorted["Latency (ms)"].mean(),
        color="red", linestyle="--", linewidth=1.2, label="Mean",
    )
    plt.title("Latency Over Time (Sequential Execution Order)",
              fontsize=12, fontweight="bold")
    plt.xlabel("Request Index (sorted by execution order)")
    plt.ylabel("Latency (ms)")
    plt.legend()
    plt.tight_layout()
    plt.savefig("latency_time_series.png", dpi=300)
    plt.close()
    print("[SUCCESS] Generated latency_time_series.png")

    # ── PLOT 4: Per-command semantic accuracy ─────────────────────────────────
    # fillna(0) ensures commands comprised entirely of NaN semantic values 
    # (e.g. 100% timeouts) are plotted as 0% rather than dropped by matplotlib.
    cmd_acc = (
        df_valid.groupby("Command ID")["Semantic Success"]
        .mean()
        .fillna(0)
        * 100
    )

    plt.figure(figsize=(10, 4))
    plt.bar(cmd_acc.index, cmd_acc.values, color="#4C72B0")
    plt.title("Semantic Accuracy Per Command",
              fontsize=12, fontweight="bold")
    plt.xlabel("Command ID")
    plt.ylabel("Success Rate (%)")
    plt.ylim(0, 110)
    plt.xticks(cmd_acc.index)
    plt.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig("per_command_accuracy.png", dpi=300)
    plt.close()
    print("[SUCCESS] Generated per_command_accuracy.png")


if __name__ == "__main__":
    generate_plots()
