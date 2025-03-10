import matplotlib.pyplot as plt
import json
from matplotlib import font_manager
import argparse
import os
from datetime import datetime, timezone, timedelta

# Add argument parser
parser = argparse.ArgumentParser(description="Render LLM benchmark chart")

# Add custom font
font_path = "../assets/fonts/sarasa-mono-sc-regular.ttf"
font_manager.fontManager.addfont(font_path)
plt.rcParams["font.family"] = "Sarasa Mono SC"

# Define benchmark paths and their max scores
benchmarks = {
    "benchmark-ball-bouncing-inside-spinning-heptagon": {
        "path": "../benchmark-ball-bouncing-inside-spinning-heptagon/scripts/benchmark-data.json",
        "max_score": 90,
        "display_name": "ball-bouncing-inside-spinning-heptagon",
    },
    "benchmark-mandelbrot-set-meet-libai": {
        "path": "../benchmark-mandelbrot-set-meet-libai/scripts/benchmark-data.json",
        "max_score": 110,
        "display_name": "mandelbrot-set-meet-libai",
    },
    "benchmark-mars-mission": {
        "path": "../benchmark-mars-mission/scripts/benchmark-data.json",
        "max_score": 65,
        "display_name": "mars-mission",
    },
    "benchmark-solar-system": {
        "path": "../benchmark-solar-system/scripts/benchmark-data.json",
        "max_score": 90,
        "display_name": "solar-system",
    },
}

# Load data from each benchmark
all_data = {}
for benchmark_key, benchmark_info in benchmarks.items():
    try:
        with open(benchmark_info["path"], "r", encoding="utf-8") as f:
            data = json.load(f)
            # Normalize scores to 100 points scale
            for entry in data:
                if entry["Final-Score"] > 0:  # Avoid division by zero
                    normalized_score = (
                        entry["Final-Score"] / benchmark_info["max_score"]
                    ) * 100
                    entry["Normalized-Score"] = round(normalized_score, 1)
                else:
                    entry["Normalized-Score"] = 0
            all_data[benchmark_key] = data
    except Exception as e:
        print(f"Error loading {benchmark_key}: {e}")
        all_data[benchmark_key] = []

# Get unique LLM names across all benchmarks
all_llms = set()
for benchmark_data in all_data.values():
    for entry in benchmark_data:
        all_llms.add(entry["LLM"])
all_llms = sorted(list(all_llms))

# Create a combined dataset with normalized scores for each LLM across all benchmarks
combined_data = []
for llm in all_llms:
    entry = {"LLM": llm}
    total_score = 0
    benchmarks_count = 0

    for benchmark_key, benchmark_data in all_data.items():
        # Find this LLM in the benchmark data
        llm_data = next((item for item in benchmark_data if item["LLM"] == llm), None)
        if llm_data:
            entry[benchmark_key] = llm_data["Normalized-Score"]
            total_score += llm_data["Normalized-Score"]
            benchmarks_count += 1
        else:
            entry[benchmark_key] = 0

    # Store total score instead of average
    entry["Final-Score"] = round(total_score, 1)

    combined_data.append(entry)

# Sort by Final-Score in descending order
combined_data.sort(key=lambda x: x["Final-Score"], reverse=True)

# Prepare data for plotting
llm_names = [entry["LLM"] for entry in combined_data]

# Colors for each benchmark
colors = {
    "benchmark-ball-bouncing-inside-spinning-heptagon": "#65318e",  # 234,85,6
    "benchmark-mandelbrot-set-meet-libai": "#674196",  # 77,67,152
    "benchmark-mars-mission": "#674598",  # 245,177,170
    "benchmark-solar-system": "#7058a3",  # 170,76,143
}

# Plotting
fig, ax = plt.subplots(figsize=(20, 6))

# Customize font size and weight for better readability
plt.rcParams["font.size"] = 10
plt.rcParams["font.weight"] = "normal"

# Initialize bottom for stacking
bottom = [0] * len(llm_names)

# Plot each benchmark with specified colors
for i, (benchmark_key, benchmark_info) in enumerate(benchmarks.items()):
    benchmark_scores = [entry.get(benchmark_key, 0) for entry in combined_data]
    bars = ax.bar(
        llm_names,
        benchmark_scores,
        bottom=bottom,
        label=benchmark_info["display_name"],
        color=colors[benchmark_key],
        edgecolor="black",
        linestyle="--",
        linewidth=0.5,
        width=0.8,
    )
    bottom = [i + j for i, j in zip(bottom, benchmark_scores)]

# Add total score labels on top of bars
for i, score in enumerate(bottom):
    ax.text(i, score, f"{combined_data[i]['Final-Score']}", ha="center", va="bottom")

# Add labels and title
ax.set_ylabel("Normalized Score (0-100)")
ax.set_title(
    "KCORES LLM Arena - Real World Coding Benchmark (Normalized to 100-point scale)\nby karminski-牙医\nhttps://github.com/KCORES/kcores-llm-arena"
)

# Add version timestamp in the bottom right corner (UTC+8)
now = datetime.now(timezone(timedelta(hours=8)))
timestamp = now.strftime("%Y-%m-%dT%H:%M:%S%z")
fig.text(0.98, 0.01, f"version: {timestamp}", ha="right", va="bottom", fontsize=8)

ax.set_ylim(0, 400)
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), reverse=True)

# Rotate x-axis labels for better readability
plt.xticks(rotation=45, ha="right")

# Show plot
plt.tight_layout()
# Save the plot as PNG file
plt.savefig(
    "llm_benchmark_results_normalized.png", dpi=300, bbox_inches="tight", pad_inches=0.5
)
plt.show()
