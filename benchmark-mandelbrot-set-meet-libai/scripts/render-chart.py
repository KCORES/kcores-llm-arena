import matplotlib.pyplot as plt
import json

# JSON data
data = [
    {
        "LLM": "DeepSeek-R1",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 5,
        "Cat-5": 2,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 5,
        "Cat-11": 5,
        "Cat-12": 5,
        "Cat-13": 5,
        "Cat-14": 3,
        "Cat-15": 0,
        "Cat-16": 5,
        "Cat-17": 20,
        "Final-Score": 90,
    },
    {
        "LLM": "DeepSeek-V3",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 5,
        "Cat-5": 4,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 4,
        "Cat-11": 5,
        "Cat-12": 0,
        "Cat-13": 5,
        "Cat-14": 3,
        "Cat-15": 3,
        "Cat-16": 5,
        "Cat-17": 13,
        "Final-Score": 82,
    },
    {"LLM": "GPT-4o-mini", "Final-Score": 0},
    {
        "LLM": "GPT-4o",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 4,
        "Cat-5": 4,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 5,
        "Cat-11": 5,
        "Cat-12": 0,
        "Cat-13": 3,
        "Cat-14": 5,
        "Cat-15": 5,
        "Cat-16": 5,
        "Cat-17": 12,
        "Final-Score": 83,
    },
    {
        "LLM": "Gemini-2.0-Flash-Lite-Preview-02-05",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 3,
        "Cat-5": 3,
        "Cat-6": 0,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 5,
        "Cat-11": 0,
        "Cat-12": 0,
        "Cat-13": 3,
        "Cat-14": 3,
        "Cat-15": 0,
        "Cat-16": 5,
        "Cat-17": 9,
        "Final-Score": 61,
    },
    {
        "LLM": "Gemini-2.0-Flash-Thinking-Experimental-01-21",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 5,
        "Cat-5": 4,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 3,
        "Cat-10": 5,
        "Cat-11": 0,
        "Cat-12": 5,
        "Cat-13": 5,
        "Cat-14": 3,
        "Cat-15": 5,
        "Cat-16": 5,
        "Cat-17": 5,
        "Final-Score": 75,
    },
    {
        "LLM": "Gemini-2.0-Flash",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 4,
        "Cat-5": 4,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 5,
        "Cat-11": 5,
        "Cat-12": 0,
        "Cat-13": 5,
        "Cat-14": 3,
        "Cat-15": 3,
        "Cat-16": 5,
        "Cat-17": 7,
        "Final-Score": 76,
    },
    {
        "LLM": "Gemini-2.0-Pro-Experimental-02-05",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 3,
        "Cat-5": 4,
        "Cat-6": 0,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 4,
        "Cat-11": 5,
        "Cat-12": 0,
        "Cat-13": 5,
        "Cat-14": 5,
        "Cat-15": 5,
        "Cat-16": 5,
        "Cat-17": 5,
        "Final-Score": 71,
    },
    {"LLM": "OpenAI-o1-mini", "Final-Score": 0},
    {
        "LLM": "OpenAI-o1",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 4,
        "Cat-5": 2,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 5,
        "Cat-11": 5,
        "Cat-12": 5,
        "Cat-13": 5,
        "Cat-14": 5,
        "Cat-15": 3,
        "Cat-16": 5,
        "Cat-17": 8,
        "Final-Score": 82,
    },
    {
        "LLM": "OpenAI-o3-mini",
        "Cat-1": 5,
        "Cat-2": 0,
        "Cat-3": 5,
        "Cat-4": 5,
        "Cat-5": 4,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 5,
        "Cat-10": 5,
        "Cat-11": 5,
        "Cat-12": 5,
        "Cat-13": 5,
        "Cat-14": 5,
        "Cat-15": 5,
        "Cat-16": 3,
        "Cat-17": 5,
        "Final-Score": 77,
    },
    {"LLM": "claude-3-opus", "Final-Score": 0},
    {
        "LLM": "claude-3.5-sonnet",
        "Cat-1": 5,
        "Cat-2": 5,
        "Cat-3": 5,
        "Cat-4": 5,
        "Cat-5": 5,
        "Cat-6": 5,
        "Cat-7": 5,
        "Cat-8": 5,
        "Cat-9": 3,
        "Cat-10": 5,
        "Cat-11": 5,
        "Cat-12": 0,
        "Cat-13": 5,
        "Cat-14": 5,
        "Cat-15": 5,
        "Cat-16": 5,
        "Cat-17": 13,
        "Final-Score": 86,
    },
]

# Extract LLM names and scores (only for LLMs with detailed scores)
scores = [entry for entry in data if "Cat-1" in entry]
# Sort scores by Final-Score in descending order
scores.sort(key=lambda x: x["Final-Score"], reverse=True)
llm_names = [entry["LLM"] for entry in scores]  # Only get names for LLMs with scores

# Prepare data for stacked bar chart
categories = [f"Cat-{i}" for i in range(1, 18)]
category_scores = {
    category: [entry.get(category, 0) for entry in scores] for category in categories
}

# Plotting
fig, ax = plt.subplots(figsize=(10, 6))

# Initialize bottom for stacking
bottom = [0] * len(llm_names)

# Plot each category
for category in categories:
    ax.bar(llm_names, category_scores[category], bottom=bottom, label=category)
    bottom = [i + j for i, j in zip(bottom, category_scores[category])]

# Add labels and title
ax.set_ylabel("Score")
ax.set_title("KCORES LLM Arena - Mandelbrot Set Meet Libai Benchmark")
ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))

# Rotate x-axis labels for better readability
plt.xticks(rotation=45, ha="right")

# Show plot
plt.tight_layout()
# Save the plot as PNG file
plt.savefig("llm_benchmark_results.png", dpi=300, bbox_inches="tight")
plt.show()
