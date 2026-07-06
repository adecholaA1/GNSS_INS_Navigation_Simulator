from pathlib import Path

def create_results_directories():

    Path("results").mkdir(exist_ok=True)
    Path("results/figures").mkdir(exist_ok=True)
    Path("results/videos").mkdir(exist_ok=True)
    Path("results/data").mkdir(exist_ok=True)
    Path("results/reports").mkdir(exist_ok=True)