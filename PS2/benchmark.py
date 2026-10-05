import csv
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from multigrid import create_multi_robot_grid
from simulation import run_plain, run_dpta

TRIALS = int(sys.argv[1]) if len(sys.argv) > 1 else 20

METRICS = [("success_rate", "Robots reaching goal (%)"),
           ("avg_path_length", "Avg path length (steps)"),
           ("makespan", "Makespan (ticks)"),
           ("vertex", "Vertex conflicts"),
           ("edge", "Edge conflicts"),
           ("replans", "Re-plans triggered"),
           ("runtime_ms", "Total run time (ms)"),
           ("nodes", "Nodes expanded")]


def average(rows, cols, obs, n):
    acc = {"Plain A*": [], "DPTA*": []}
    for seed in range(TRIALS):
        grid, s, g = create_multi_robot_grid(rows, cols, obs / 100, n,
                                             seed=1000 + seed)
        if grid is None:
            continue
        acc["Plain A*"].append(run_plain(grid, s, g)["metrics"])
        acc["DPTA*"].append(run_dpta(grid, s, g)["metrics"])
    return {m: {k: sum(x[k] for x in v) / max(1, len(v)) for k, _ in METRICS}
            for m, v in acc.items()}


def experiment(title, xlabel, values, cfg, fname, csv_rows):
    data = {"Plain A*": [], "DPTA*": []}
    for v in values:
        print(f"  {title}: {xlabel} = {v}")
        r = average(*cfg(v))
        for m in data:
            data[m].append(r[m])
            csv_rows.append([title, v, m] + [round(r[m][k], 3) for k, _ in METRICS])

    fig, axes = plt.subplots(2, 4, figsize=(20, 8))
    for ax, (key, name) in zip(axes.flat, METRICS):
        for m, style in (("Plain A*", "o--"), ("DPTA*", "s-")):
            ax.plot(values, [d[key] for d in data[m]], style, label=m)
        ax.set_xlabel(xlabel)
        ax.set_title(name)
        ax.grid(alpha=0.3)
        ax.legend()
    fig.suptitle(f"Plain A* vs DPTA*  -  {title}  ({TRIALS} random grids per point)")
    fig.tight_layout()
    fig.savefig(fname, dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    rows_out = []
    experiment("varying number of robots (20x20, 25% obstacles)", "robots",
               [2, 4, 6, 8, 10, 15, 20], lambda n: (20, 20, 25, n),
               "results_robots.png", rows_out)
    experiment("varying grid size (8 robots, 25% obstacles)", "grid size N (NxN)",
               [10, 15, 20, 30, 40], lambda s: (s, s, 25, 8),
               "results_gridsize.png", rows_out)
    experiment("varying obstacle density (20x20, 8 robots)", "obstacles (%)",
               [10, 20, 25, 30, 35], lambda o: (20, 20, o, 8),
               "results_obstacles.png", rows_out)

    with open("results.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["experiment", "x", "method"] + [k for k, _ in METRICS])
        w.writerows(rows_out)
    print("Saved results_*.png and results.csv")