#!/bin/bash

# Store the original directory path
ORIGINAL_DIR=$(pwd)

# Print current directory and list benchmark directories for debugging
echo "Current directory: $(pwd)"
echo "Available benchmark directories:"
find . -type d -name "benchmark-*" | sort

# Find all benchmark-*/scripts/render-chart.py files and run them
echo "Searching for render-chart.py scripts..."
SCRIPTS=$(find . -path "*/benchmark-*/scripts/render-chart.py" -o -path "./benchmark-*/scripts/render-chart.py")

if [ -z "$SCRIPTS" ]; then
    echo "No render-chart.py scripts found! Trying different pattern..."
    SCRIPTS=$(find . -name "render-chart.py" | grep "benchmark-")
fi

for chart_script in $SCRIPTS; do
    echo "Running $chart_script..."
    
    # Get the directory of the script
    SCRIPT_DIR=$(dirname "$chart_script")
    BENCHMARK_DIR=$(dirname "$SCRIPT_DIR")
    
    echo "Script directory: $SCRIPT_DIR"
    echo "Benchmark directory: $BENCHMARK_DIR"
    
    # Change to the scripts directory and run the script
    cd "$SCRIPT_DIR"
    python3 "./render-chart.py"
    
    # Return to the original directory
    cd "$ORIGINAL_DIR"
    
    echo "Completed $chart_script"
    echo "-----------------------"
done

# Finally, run the coding benchmark chart renderer
echo "Running render-coding-benchmark-chart.py..."
if [ -f "./scripts/render-coding-benchmark-chart.py" ]; then
    python3 "./scripts/render-coding-benchmark-chart.py"
else
    echo "Error: render-coding-benchmark-chart.py not found in current directory!"
    echo "Current directory: $(pwd)"
    echo "Files in current directory:"
    ls -la
fi

echo "All chart rendering completed!"
