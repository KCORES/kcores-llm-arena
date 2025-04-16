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
    "point_1": "1 - 展示太阳",
    "point_2": "2 - 展示地球, 火星, 以及宇宙飞船",
    "point_3": "3 - 展示地球, 火星绕太阳公转",
    "point_4": "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹",
    "point_5": "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星",
    "point_6": "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球",
    "point_7": "7 - 轨道比例",
    "point_8": "8 - 发射周期",
    "point_9": "9 - 返回窗口",
    "point_10": "10 - 三维空间表现",
    "point_11": "11 - 比例尺",
    "point_12": "12 - 图例标注",
    "point_13": "13 - 动画流畅性",
}

# 使用JSON中实际的键名
categories = list(category_mapping.keys())
category_labels = list(category_mapping.values())

# 更新颜色配置 (使用13种不同的颜色)
colors = [
    "#f4b3c2",
    "#f09199",
    "#ee827c",
    "#f0908d",
    "#f2a0a1",
    "#efab93",
    "#f5b199",
    "#f5b1aa",
    "#f6bfbc",
    "#e4d2d8",
    "#e9dfe5",
    "#fdeff2",
    "#fef4f4",
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
    "KCORES LLM Arena - Mars Mission Animation Benchmark\nby karminski-牙医\nhttps://github.com/KCORES/kcores-llm-arena"
)

ax.set_ylim(0, 110)
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))

# Rotate x-axis labels for better readability
plt.xticks(rotation=45, ha="right")

# Show plot
plt.tight_layout()
# Save the plot as PNG file
plt.savefig(
    "llm-benchmark-results-mars-mission.png",
    dpi=300,
    bbox_inches="tight",
    pad_inches=0.5,
)
plt.show()
