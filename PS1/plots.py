import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv("summary.csv")
print(df)
plt.figure(figsize=(8, 5))
plt.bar(
    df["heuristic"],
    df["average_nodes_expanded"]
)
plt.xlabel("Heuristic")
plt.ylabel(
    "Average Nodes Expanded"
)
plt.title(
    "Average Nodes Expanded by Heuristic"
)
plt.tight_layout()
plt.savefig(
    "average_nodes_expanded.png",
    dpi=300
)
plt.show()
plt.figure(figsize=(8, 5))
plt.bar(
    df["heuristic"],
    df["average_execution_time"]
)
plt.xlabel("Heuristic")
plt.ylabel(
    "Average Execution Time (seconds)"
)
plt.title(
    "Average Execution Time by Heuristic"
)
plt.tight_layout()
plt.savefig(
    "average_execution_time.png",
    dpi=300
)
plt.show()
plt.figure(figsize=(8, 5))
plt.bar(
    df["heuristic"],
    df["average_path_length"]
)
plt.xlabel("Heuristic")
plt.ylabel(
    "Average Path Length"
)
plt.title(
    "Average Path Length by Heuristic"
)
plt.tight_layout()
plt.savefig(
    "average_path_length.png",
    dpi=300
)
plt.show()
print("\nGraphs generated successfully!")