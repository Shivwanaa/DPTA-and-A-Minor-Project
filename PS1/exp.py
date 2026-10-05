import csv
import sys
import os
from grid import create_grid
from astar import (
    astar,
    manhattan,
    euclidean,
    chebyshev
)
NUM_TRIALS = 100
OUTPUT_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "results.csv"
)
heuristics = {
    "Manhattan": manhattan,
    "Euclidean": euclidean,
    "Chebyshev": chebyshev
}
def run_experiments(
    rows,
    cols,
    obstacle_probability
):
    results = []
    for trial in range(
        1,
        NUM_TRIALS + 1
    ):
        grid, start, goal = create_grid(
            rows,
            cols,
            obstacle_probability
        )
        for name, heuristic in heuristics.items():
            result = astar(
                grid,
                start,
                goal,
                heuristic
            )
            results.append({
                "trial": trial,
                "heuristic": name,
                "rows": rows,
                "cols": cols,
                "obstacle_probability":
                    obstacle_probability,
                "start_row": start[0],
                "start_col": start[1],
                "goal_row": goal[0],
                "goal_col": goal[1],
                "path_length":
                    result["path_length"],
                "nodes_expanded":
                    result["nodes_expanded"],
                "execution_time":
                    result["time"],
                "success":
                    result["path"] is not None
            })
        print(
            f"Completed trial "
            f"{trial}/{NUM_TRIALS}"
        )
    fieldnames = [
        "trial",
        "heuristic",
        "rows",
        "cols",
        "obstacle_probability",
        "start_row",
        "start_col",
        "goal_row",
        "goal_col",
        "path_length",
        "nodes_expanded",
        "execution_time",
        "success"
    ]
    with open(
        OUTPUT_FILE,
        "w",
        newline=""
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames
        )
        writer.writeheader()
        writer.writerows(results)
    print()
    print("Experiments completed.")
    print(
        f"Results saved to {OUTPUT_FILE}"
    )
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(
            "Usage: python exp.py "
            "<rows> <cols> <obstacle_probability>"
        )
        sys.exit(1)
    try:
        rows = int(sys.argv[1])
        cols = int(sys.argv[2])
        obstacle_probability = float(
            sys.argv[3]
        )
    except ValueError:
        print(
            "Invalid experiment parameters."
        )
        sys.exit(1)
    rows = max(
        5,
        min(50, rows)
    )
    cols = max(
        5,
        min(50, cols)
    )
    obstacle_probability = max(
        0,
        min(0.90, obstacle_probability)
    )
    run_experiments(
        rows,
        cols,
        obstacle_probability
    )

