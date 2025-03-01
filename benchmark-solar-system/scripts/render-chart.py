import matplotlib.pyplot as plt
import json
from matplotlib import font_manager
import argparse

# Add argument parser
parser = argparse.ArgumentParser(description="Render LLM benchmark chart")

# Add custom font
font_path = "../../assets/fonts/sarasa-mono-sc-regular.ttf"
font_manager.fontManager.addfont(font_path)
plt.rcParams["font.family"] = "Sarasa Mono SC"

# JSON data

with open("benchmark-data.json", "r", encoding="utf-8") as f:
    data = json.load(f)

# Extract LLM names and scores (including all LLMs)
scores = data  # Use all data entries

# Sort scores by Final-Score in descending order
scores.sort(key=lambda x: x["Final-Score"], reverse=True)
llm_names = [entry["LLM"] for entry in scores]

# Prepare data for stacked bar chart with mapping between point keys and display labels
category_mapping = {
    "point_1": "1-使用 canvas 绘制动画",
    "point_2": "2-全屏展示动画",
    "point_3": "3-所有代码放在同一个HTML文件里面",
    "point_4": "4-太阳大小及展示",
    "point_5": "5-九大行星展示",
    "point_6": "6-九大行星大小展示",
    "point_7": "7-九大行星排列正确",
    "point_8": "8-九大行星绕太阳公转",
    "point_9": "9-九大行星运动轨迹",
    "point_10": "10-九大行星运动轨迹不重叠",
    "point_11": "11-九大行星颜色渲染",
    "point_12": "12-九大行星颜色渲染美观度",
    "point_13": "13-九大行星名称标签",
    "point_14": "14-地球运动时间",
    "point_15": "15-其他行星运动时间",
    "point_16": "16-指示器计算正确",
    "point_17": "17-指示器样式正确",
    "point_18": "18-动画流畅性",
}

# 使用JSON中实际的键名
categories = list(category_mapping.keys())
category_labels = list(category_mapping.values())

# 更新颜色配置 (使用18种不同的颜色)
colors = [
    "#895b8a",
    "#824880",
    "#915c8b",
    "#9d5b8b",
    "#7a4171",
    "#bc64a4",
    "#b44c97",
    "#aa4c8f",
    "#cc7eb1",
    "#cca6bf",
    "#c4a3bf",
    "#e7e7eb",
    "#dcd6d9",
    "#d3cfd9",
    "#c8c2c6",
    "#a6a5c4",
    "#a69abd",
    "#95859c",
]

category_scores = {
    category: [entry.get(category, 0) for entry in scores] for category in categories
}

# Plotting
fig, ax = plt.subplots(figsize=(20, 6))

# Customize font size and weight for better readability
plt.rcParams["font.size"] = 10
plt.rcParams["font.weight"] = "normal"

# Initialize bottom for stacking
bottom = [0] * len(llm_names)

# Plot each category with specified colors
for i, category in enumerate(categories):
    bars = ax.bar(
        llm_names,
        [entry.get(category, 0) for entry in scores],
        bottom=bottom,
        label=category_mapping[category],  # 使用映射的标签
        color=colors[i],
        edgecolor="black",
        linestyle="--",
        linewidth=0.5,
        width=0.8,
    )
    bottom = [
        i + j for i, j in zip(bottom, [entry.get(category, 0) for entry in scores])
    ]

# Add total score labels on top of bars
for i, score in enumerate(bottom):  # 使用 bottom 作为最终高度
    ax.text(i, score, f"{scores[i]['Final-Score']}", ha="center", va="bottom")

# Add labels and title
ax.set_ylabel("Score")
ax.set_title(
    "KCORES LLM Arena - Solar System Animation Benchmark\nby karminski-牙医\nhttps://github.com/KCORES/kcores-llm-arena"
)

ax.set_ylim(0, 110)
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))

# Rotate x-axis labels for better readability
plt.xticks(rotation=45, ha="right")

# Show plot
plt.tight_layout()
# Save the plot as PNG file
plt.savefig("llm_benchmark_results.png", dpi=300, bbox_inches="tight", pad_inches=0.5)
plt.show()
