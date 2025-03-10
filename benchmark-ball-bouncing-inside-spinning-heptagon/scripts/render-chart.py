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
    "point_1": "1-所有代码都实现在一个python文件里面",
    "point_2": "2-使用指定的 python 库进行编码",
    "point_3": "3-展示 20 个小球",
    "point_4": "4-所有小球大小一致",
    "point_5": "5-小球上的数字展示正确",
    "point_6": "6-小球从7边形中间掉落开始",
    "point_7": "7-小球颜色正确",
    "point_8": "8-小球的碰撞运算正确",
    "point_9": "9-小球的摩擦运算正确",
    "point_10": "10-小球的重力运算正确",
    "point_11": "11-小球的弹性运算正确",
    "point_12": "12-小球上的数字旋转",
    "point_13": "13-小球不会重叠",
    "point_14": "14-小球不会超出七边形",
    "point_15": "15-小球的渲染美观度",
    "point_16": "16-七边形展示正确",
    "point_17": "17-七边形旋转正确",
    "point_18": "18-动画流畅性",
}

# 使用JSON中实际的键名
categories = list(category_mapping.keys())
category_labels = list(category_mapping.values())

# 更新颜色配置 (使用18种不同的颜色)
colors = [
    "#f8b862",
    "#f6ad49",
    "#f39800",
    "#f08300",
    "#ec6d51",
    "#ee7948",
    "#ed6d3d",
    "#ec6800",
    "#ec6800",
    "#ee7800",
    "#eb6238",
    "#ea5506",
    "#ea5506",
    "#eb6101",
    "#e49e61",
    "#e45e32",
    "#e17b34",
    "#dd7a56",
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
    "KCORES LLM Arena - Ball Bouncing Inside Spinning Heptagon Benchmark\nby karminski-牙医\nhttps://github.com/KCORES/kcores-llm-arena"
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
