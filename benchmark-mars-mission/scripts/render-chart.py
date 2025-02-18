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

data = [
    {
        "LLM": "Claude-3.5-sonnet",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 3,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 5,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 5,
        "7 - 轨道比例": 3,
        "8 - 发射周期": 3,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 5,
        "11 - 比例尺": 3,
        "12 - 图例标注": 3,
        "13 - 动画流畅性": 5,
        "Final-Score": 45,
    },
    {
        "LLM": "DeepSeek-R1",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 3,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 5,
        "8 - 发射周期": 0,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 5,
        "12 - 图例标注": 0,
        "13 - 动画流畅性": 5,
        "Final-Score": 28,
    },
    {
        "LLM": "DeepSeek-V3",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 0,
        "3 - 展示地球, 火星绕太阳公转": 0,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 5,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 3,
        "8 - 发射周期": 0,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 3,
        "12 - 图例标注": 3,
        "13 - 动画流畅性": 5,
        "Final-Score": 19,
    },
    {
        "LLM": "Gemini-2.0-Flash-Lite-Preview-02-05",
        "1 - 展示太阳": 5,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 3,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 3,
        "8 - 发射周期": 0,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 3,
        "12 - 图例标注": 5,
        "13 - 动画流畅性": 0,
        "Final-Score": 29,
    },
    {
        "LLM": "Gemini-2.0-Flash-Thinking-Experimental-01-21",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 3,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 3,
        "8 - 发射周期": 0,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 3,
        "12 - 图例标注": 3,
        "13 - 动画流畅性": 5,
        "Final-Score": 27,
    },
    {
        "LLM": "Gemini-2.0-Flash",
        "1 - 展示太阳": 5,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 5,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 3,
        "8 - 发射周期": 3,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 3,
        "12 - 图例标注": 5,
        "13 - 动画流畅性": 5,
        "Final-Score": 39,
    },
    {
        "LLM": "Gemini-2.0-Pro-Experimental-02-05",
        "1 - 展示太阳": 5,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 3,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 5,
        "8 - 发射周期": 3,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 5,
        "12 - 图例标注": 5,
        "13 - 动画流畅性": 5,
        "Final-Score": 41,
    },
    {
        "LLM": "GPT-4o",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 0,
        "3 - 展示地球, 火星绕太阳公转": 0,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 0,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 0,
        "8 - 发射周期": 0,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 3,
        "12 - 图例标注": 0,
        "13 - 动画流畅性": 0,
        "Final-Score": 3,
    },
    {
        "LLM": "Grok-2",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 3,
        "3 - 展示地球, 火星绕太阳公转": 0,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 5,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 3,
        "8 - 发射周期": 0,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 3,
        "12 - 图例标注": 3,
        "13 - 动画流畅性": 0,
        "Final-Score": 17,
    },
    {
        "LLM": "Grok-3",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 5,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 5,
        "8 - 发射周期": 3,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 5,
        "12 - 图例标注": 3,
        "13 - 动画流畅性": 5,
        "Final-Score": 36,
    },
    {
        "LLM": "OpenAI-o1-mini",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 0,
        "3 - 展示地球, 火星绕太阳公转": 0,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 0,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 0,
        "8 - 发射周期": 0,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 3,
        "12 - 图例标注": 0,
        "13 - 动画流畅性": 0,
        "Final-Score": 3,
    },
    {
        "LLM": "OpenAI-o1",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 3,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 5,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 5,
        "7 - 轨道比例": 5,
        "8 - 发射周期": 3,
        "9 - 返回窗口": 3,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 5,
        "12 - 图例标注": 3,
        "13 - 动画流畅性": 5,
        "Final-Score": 47,
    },
    {
        "LLM": "OpenAI-o3-mini",
        "1 - 展示太阳": 5,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 0,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 5,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 0,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 5,
        "8 - 发射周期": 3,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 5,
        "12 - 图例标注": 5,
        "13 - 动画流畅性": 0,
        "Final-Score": 33,
    },
    {
        "LLM": "Qwen-2.5-Max",
        "1 - 展示太阳": 0,
        "2 - 展示地球, 火星, 以及宇宙飞船": 5,
        "3 - 展示地球, 火星绕太阳公转": 5,
        "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹": 3,
        "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星": 5,
        "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球": 0,
        "7 - 轨道比例": 5,
        "8 - 发射周期": 3,
        "9 - 返回窗口": 0,
        "10 - 三维空间表现": 0,
        "11 - 比例尺": 5,
        "12 - 图例标注": 3,
        "13 - 动画流畅性": 5,
        "Final-Score": 39,
    },
]

# Extract LLM names and scores (including all LLMs)
scores = data  # Use all data entries

# Sort scores by Final-Score in descending order
scores.sort(key=lambda x: x["Final-Score"], reverse=True)
llm_names = [entry["LLM"] for entry in scores]

# Prepare data for stacked bar chart
categories = [
    "1 - 展示太阳",
    "2 - 展示地球, 火星, 以及宇宙飞船",
    "3 - 展示地球, 火星绕太阳公转",
    "4 - 展示地球, 火星, 以及宇宙飞船的运动轨迹",
    "5 - 宇宙飞船从地球发射, 并沿椭圆轨道成功着陆火星",
    "6 - 宇宙飞船从火星返回, 并沿椭圆轨道成功着陆地球",
    "7 - 轨道比例",
    "8 - 发射周期",
    "9 - 返回窗口",
    "10 - 三维空间表现",
    "11 - 比例尺",
    "12 - 图例标注",
    "13 - 动画流畅性",
]

# 更新颜色配置 (使用13种不同的颜色)
colors = [
    "#fef4f4",
    "#fdeff2",
    "#e9dfe5",
    "#e4d2d8",
    "#f6bfbc",
    "#f5b1aa",
    "#f5b199",
    "#efab93",
    "#f2a0a1",
    "#f0908d",
    "#ee827c",
    "#f09199",
    "#f4b3c2",
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
        label=category,
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
plt.savefig("llm_benchmark_results.png", dpi=300, bbox_inches="tight", pad_inches=0.5)
plt.show()
