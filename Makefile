# Makefile for kcores-LLM-Arena

# Default target
.PHONY: all
all: install-deps render-charts

# Install Python dependencies from requirements.txt
.PHONY: install-deps
install-deps:
	@echo "Installing Python dependencies..."
	pip install -r scripts/requirements.txt

# Run benchmark chart rendering script
.PHONY: render-charts
render-charts:
	@echo "Rendering benchmark charts..."
	bash scripts/render-all-benchmark-chart.sh

# Clean target (can be expanded as needed)
.PHONY: clean
clean:
	@echo "Cleaning up..."
	find . -name "*.pyc" -delete
	find . -name "__pycache__" -type d -exec rm -rf {} +

# Help target
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all          : Install dependencies and render charts (default)"
	@echo "  install-deps : Install Python dependencies"
	@echo "  render-charts: Render benchmark charts"
	@echo "  clean        : Clean up Python cache files"
	@echo "  help         : Show this help message"
