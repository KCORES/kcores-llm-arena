import matplotlib.pyplot as plt
import json
from matplotlib import font_manager

font_path = "sarasa-mono-sc-regular.ttf"
font_manager.fontManager.addfont(font_path)
plt.rcParams["font.family"] = "Sarasa Mono SC"

# JSON data
data = [
    {
        "LLM": "DeepSeek-R1",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 5,
        "5 - Mandelbrot Set 的主要图形大小": 2,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 5,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 3,
        "15 - 左上角指示器计算和展示正确": 0,
        "16 - 指示器样式": 5,
        "Final-Score": 70,
    },
    {
        "LLM": "DeepSeek-V3",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 5,
        "5 - Mandelbrot Set 的主要图形大小": 4,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 4,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 0,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 3,
        "15 - 左上角指示器计算和展示正确": 3,
        "16 - 指示器样式": 5,
        "Final-Score": 69,
    },
    {"LLM": "GPT-4o-mini", "Final-Score": 0},
    {
        "LLM": "GPT-4o",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 4,
        "5 - Mandelbrot Set 的主要图形大小": 4,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 0,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 3,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 5,
        "15 - 左上角指示器计算和展示正确": 5,
        "16 - 指示器样式": 5,
        "Final-Score": 71,
    },
    {
        "LLM": "Gemini-2.0-Flash-Lite-Preview-02-05",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 3,
        "5 - Mandelbrot Set 的主要图形大小": 3,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 0,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 0,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 0,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 3,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 3,
        "15 - 左上角指示器计算和展示正确": 0,
        "16 - 指示器样式": 5,
        "Final-Score": 52,
    },
    {
        "LLM": "Gemini-2.0-Flash-Thinking-Experimental-01-21",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 5,
        "5 - Mandelbrot Set 的主要图形大小": 4,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 3,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 0,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 5,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 3,
        "15 - 左上角指示器计算和展示正确": 5,
        "16 - 指示器样式": 5,
        "Final-Score": 70,
    },
    {
        "LLM": "Gemini-2.0-Flash",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 4,
        "5 - Mandelbrot Set 的主要图形大小": 4,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 0,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 3,
        "15 - 左上角指示器计算和展示正确": 3,
        "16 - 指示器样式": 5,
        "Final-Score": 69,
    },
    {
        "LLM": "Gemini-2.0-Pro-Experimental-02-05",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 3,
        "5 - Mandelbrot Set 的主要图形大小": 4,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 0,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 4,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 0,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 5,
        "15 - 左上角指示器计算和展示正确": 5,
        "16 - 指示器样式": 5,
        "Final-Score": 66,
    },
    {"LLM": "OpenAI-o1-mini", "Final-Score": 0},
    {
        "LLM": "OpenAI-o1",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 4,
        "5 - Mandelbrot Set 的主要图形大小": 2,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 5,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 5,
        "15 - 左上角指示器计算和展示正确": 3,
        "16 - 指示器样式": 5,
        "Final-Score": 74,
    },
    {
        "LLM": "OpenAI-o3-mini",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 0,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 5,
        "5 - Mandelbrot Set 的主要图形大小": 4,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 5,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 5,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 5,
        "15 - 左上角指示器计算和展示正确": 5,
        "16 - 指示器样式": 3,
        "Final-Score": 72,
    },
    {"LLM": "claude-3-opus", "Final-Score": 0},
    {
        "LLM": "claude-3.5-sonnet",
        "1 - 使用 canvas 绘制动画": 5,
        "2 - 全屏展示动画": 5,
        "3 - 所有代码放在同一个HTML文件里面": 5,
        "4 - Mandelbrot Set 图形美观度": 5,
        "5 - Mandelbrot Set 的主要图形大小": 5,
        "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%": 5,
        "7 - 总计渲染200次": 5,
        "8 - 总计渲染200次后重置并循环": 5,
        "9 - 李白诗书写正确": 3,
        "10 - 李白诗处理": 5,
        "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空": 5,
        "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处": 0,
        "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ": 5,
        "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅": 5,
        "15 - 左上角指示器计算和展示正确": 5,
        "16 - 指示器样式": 5,
        "Final-Score": 73,
    },
]

# Extract all LLM entries and sort them by Final-Score
scores = data  # Use all data entries
scores.sort(key=lambda x: x["Final-Score"], reverse=True)
llm_names = [entry["LLM"] for entry in scores]

# Prepare data for stacked bar chart
categories = [
    "1 - 使用 canvas 绘制动画",
    "2 - 全屏展示动画",
    "3 - 所有代码放在同一个HTML文件里面",
    "4 - Mandelbrot Set 图形美观度",
    "5 - Mandelbrot Set 的主要图形大小",
    "6 - Mandelbrot Set 的主要图形每渲染一次每帧放大 0.5%",
    "7 - 总计渲染200次",
    "8 - 总计渲染200次后重置并循环",
    "9 - 李白诗书写正确",
    "10 - 李白诗处理",
    "11 - Mandelbrot Set 的 Main cardioid and period bulbs 部分留空",
    "12 - 动画的中心应使始终为 Main cardioid and period bulbs 的交界处",
    "13 - 动画字体大小8px, 字体渲染排列也是8px，无间距 ",
    "14 - 字符从 mandelbrot set的最外围使用最深的颜色，然后依次变浅",
    "15 - 左上角指示器计算和展示正确",
    "16 - 指示器样式",
]
category_scores = {
    category: [entry.get(category, 0) for entry in scores] for category in categories
}

# Plotting
fig, ax = plt.subplots(figsize=(20, 6))  # 增加整体宽度到20

# Initialize bottom for stacking
bottom = [0] * len(llm_names)

# Plot each category
for category in categories:
    ax.bar(llm_names, category_scores[category], bottom=bottom, label=category)
    bottom = [i + j for i, j in zip(bottom, category_scores[category])]

# Add total scores on top of each bar
final_scores = [entry["Final-Score"] for entry in scores]
for i, score in enumerate(final_scores):
    ax.text(i, bottom[i], f"{score}", ha="center", va="bottom")

# Add labels and title
ax.set_ylabel("Score")
ax.set_title("KCORES LLM Arena - Mandelbrot Set Meet Libai Benchmark (No FPS)")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))  # 调整图例位置

# Rotate x-axis labels for better readability
plt.xticks(rotation=45, ha="right")

# 调整布局，为图例留出足够空间
plt.tight_layout()
plt.subplots_adjust(right=0.8)  # 为右侧图例预留空间

# Show plot
plt.tight_layout()
# Save the plot as PNG file
plt.savefig("llm_benchmark_results_no_fps.png", dpi=300, bbox_inches="tight")
plt.show()
