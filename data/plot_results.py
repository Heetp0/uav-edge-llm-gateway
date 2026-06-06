#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import os

def generate_plots():
    # Load the data
    csv_path = os.path.expanduser("~/quantization_benchmark_results.csv")
    if not os.path.exists(csv_path):
        print(f"Error: Data file not found at {csv_path}. Run benchmark first.")
        return

    df = pd.read_csv(csv_path)

    # Clean data: Filter out TIMEOUTs for latency math
    df_valid = df[df['Latency (ms)'] != 'TIMEOUT'].copy()
    df_valid['Latency (ms)'] = pd.to_numeric(df_valid['Latency (ms)'])

    # Calculate Accuracy Percentages
    total_runs = len(df)
    syntax_acc = (df['Syntactic Success'] == 'True').sum() / total_runs * 100
    semantic_acc = (df['Semantic Success'] == 'True').sum() / total_runs * 100

    # ---------------------------------------------------------
    # PLOT 1: Latency Distribution (IEEE standard formatting)
    # ---------------------------------------------------------
    plt.figure(figsize=(6, 8))
    plt.boxplot(df_valid['Latency (ms)'], patch_artist=True, boxprops=dict(facecolor='lightgrey'))
    plt.title('Edge Inference Latency (qwen2.5:3b - 4-bit Quantized)', fontsize=12, fontweight='bold')
    plt.ylabel('Latency (ms)', fontsize=12)
    plt.xticks([1], ['WSL2 Edge Environment'])
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Save high-res for LaTeX
    plt.tight_layout()
    plt.savefig('latency_distribution.png', dpi=300)
    print("[SUCCESS] Generated latency_distribution.png")

    # ---------------------------------------------------------
    # PLOT 2: System Accuracy
    # ---------------------------------------------------------
    plt.figure(figsize=(7, 5))
    bars = plt.bar(['Syntactic JSON Success', 'Semantic Spatial Success'], [syntax_acc, semantic_acc], color=['#4C72B0', '#55A868'])
    plt.title('LLM Instruction Translation Accuracy', fontsize=12, fontweight='bold')
    plt.ylabel('Success Rate (%)', fontsize=12)
    plt.ylim(0, 110)

    # Add percentage labels on top of bars
    for bar in bars:
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, yval + 2, f'{yval:.1f}%', ha='center', va='bottom', fontsize=11)

    plt.tight_layout()
    plt.savefig('accuracy_metrics.png', dpi=300)
    print("[SUCCESS] Generated accuracy_metrics.png")

if __name__ == "__main__":
    generate_plots()
