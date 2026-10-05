import csv
import matplotlib.pyplot as plt
data = {}
with open("robot_results.csv", "r") as file:
    reader = csv.reader(file)
    for row in reader:
        if len(row) >= 3:
            data[row[0]] = row[1:]
algorithms = ["Plain A*", "DPTA*"]
def get_values(metric, converter=float):
    return [converter(data[metric][0]), converter(data[metric][1])]
# Plot 1: Path and execution performance
metrics = ["Avg path length", "Avg arrival time", "Makespan (ticks)"]
values = [get_values("Avg path length"), get_values("Avg arrival time"), get_values("Makespan (ticks)", int)]
x = range(len(metrics))
width = 0.35
plt.figure(figsize=(10, 6))
plt.bar([i - width / 2 for i in x], [v[0] for v in values], width, label="Plain A*")
plt.bar([i + width / 2 for i in x], [v[1] for v in values], width, label="DPTA*")
plt.xticks(list(x), metrics)
plt.ylabel("Value")
plt.title("Path and Execution Performance")
plt.legend()
plt.grid(axis="y", linestyle="--", alpha=0.4)
plt.tight_layout()
plt.savefig("01_path_execution_comparison.png", dpi=300, bbox_inches="tight")
plt.show()
# Plot 2: Conflict handling
metrics = ["Predicted conflicts", "Vertex conflicts", "Edge conflicts", "Re-plans"]
plain = [int(data[m][0]) for m in metrics]
dpta = [int(data[m][1]) for m in metrics]
x = range(len(metrics))
plt.figure(figsize=(10, 6))
plt.bar([i - width / 2 for i in x], plain, width, label="Plain A*")
plt.bar([i + width / 2 for i in x], dpta, width, label="DPTA*")
plt.xticks(list(x), metrics)
plt.ylabel("Count")
plt.title("Conflict Handling Comparison")
plt.legend()
plt.grid(axis="y", linestyle="--", alpha=0.4)
plt.tight_layout()
plt.savefig("02_conflict_comparison.png", dpi=300, bbox_inches="tight")
plt.show()
# Plot 3: Computational cost
metrics = ["Planning time (ms)", "Total run time (ms)"]
plain = [float(data[m][0]) for m in metrics]
dpta = [float(data[m][1]) for m in metrics]
x = range(len(metrics))
plt.figure(figsize=(9, 6))
plt.bar([i - width / 2 for i in x], plain, width, label="Plain A*")
plt.bar([i + width / 2 for i in x], dpta, width, label="DPTA*")
plt.xticks(list(x), metrics)
plt.ylabel("Time (ms)")
plt.title("Computational Cost Comparison")
plt.legend()
plt.grid(axis="y", linestyle="--", alpha=0.4)
plt.tight_layout()
plt.savefig("03_computational_cost_comparison.png", dpi=300, bbox_inches="tight")
plt.show()
# Plot 4: Nodes expanded
algorithms = ["Plain A*", "DPTA*"]
nodes = [int(data["Nodes expanded"][0]), int(data["Nodes expanded"][1])]
plt.figure(figsize=(8, 6))
plt.bar(algorithms, nodes)
plt.ylabel("Nodes expanded")
plt.title("Search Effort Comparison")
plt.grid(axis="y", linestyle="--", alpha=0.4)
plt.tight_layout()
plt.savefig("04_nodes_expanded_comparison.png", dpi=300, bbox_inches="tight")
plt.show()
