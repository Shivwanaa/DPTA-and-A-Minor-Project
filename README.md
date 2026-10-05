# Robot Path Finding on a Grid

**A\* heuristic comparison and a multi-robot planner (DPTA\*)**

Minor project for **CSMI17 – Artificial Intelligence**
Department of Electronics and Communication Engineering, National Institute of Technology, Tiruchirappalli – 620 015
Faculty: **Dr. Usha K Ruthika**

| Name | Roll No. |
| --- | --- |
| Bhavani Sree Arunachalam | 108123024 |
| Shivwanaa Riddhii V | 108123116 |

**Tech stack:** Python · Pygame (simulator UI) · Matplotlib (graphs) · CSV (experiment results)

---

## Table of Contents

1. [Overview](#overview)
2. [Problem Statements](#problem-statements)
3. [Assumptions](#assumptions)
4. [Algorithms](#algorithms)
5. [Experimental Setup](#experimental-setup)
6. [Results](#results)
7. [Installation](#installation)
8. [Usage](#usage)
9. [Project Structure](#project-structure)
10. [Conclusion](#conclusion)
11. [References](#references)

---

## Overview

This project studies how robots find their way across a grid with obstacles, such as a warehouse floor or a parking lot with parked vehicles.

- **Problem 1:** A single robot is solved with A\* search. Three heuristics (Manhattan, Euclidean, Chebyshev) are compared on 100 random grids.
- **Problem 2:** Many robots share one grid, each with its own start and goal. Plain A\* leads to collisions, so we propose **DPTA\*** (*Dynamic-Priority Time-aware A\**), which plans conflict-free paths up front.

An interactive Pygame simulator lets you watch both problems run step by step.

---

## Problem Statements

### Problem 1: Robot Path-Finding

A robot starts in one cell of a grid and must reach a goal cell while avoiding obstacles. Solve it with A\*, use at least three heuristics, and compare their performance over many randomly generated grids (random obstacles, random start, random goal).

### Problem 2: Multi-Robot Path-Finding

Same setting, but with several robots, each with its own start and goal. Identify the additional challenges of the multi-robot case, propose a method that handles some of them, and show with experiments (different obstacle layouts, grid sizes and numbers of robots) that it beats plain A\*.

Two kinds of clashes appear once there are multiple robots:

- **Vertex conflict:** two robots occupy the same cell at the same time.
- **Edge (swap) conflict:** two robots in neighbouring cells swap places in one step.

---

## Assumptions

**Both problems**

- The map is a grid; each cell is free (`0`) or blocked (`1`).
- Obstacles (parked vehicles, pillars, shelves) are fixed for the whole run.
- Movement is **4-directional** (up, down, left, right), with no diagonals.
- Every move costs 1, so path cost equals the number of moves.
- One robot occupies one cell, and the robot knows the full map.
- A goal may be unreachable; such runs are counted as failures, not hidden.

**Problem 2 only**

- All robots move in synchronised time steps; each step is a move or a **wait**.
- Starts and goals are free cells.
- A robot that reaches its goal **stays parked** and acts as an obstacle for the others.
- Plans are made before execution. If a robot cannot find a path, the planner reorders the robots and retries, **up to 7 attempts**.

**Simulator limits:** grid side of 5 to 50 cells, obstacle density up to 90%.

---

## Algorithms

### A\* search

For each cell `n`, A\* keeps `f(n) = g(n) + h(n)`, where `g` is the real cost so far and `h` is the heuristic estimate of the remaining cost. A min-heap always expands the cell with the smallest `f`. The path is rebuilt by following parent pointers back from the goal.

### Heuristics (Problem 1)

With current cell `(r, c)` and goal `(rg, cg)`:

| Heuristic | Formula | Notes |
| --- | --- | --- |
| **Manhattan** | `\|r − rg\| + \|c − cg\|` | Exact fit for 4-direction movement |
| **Euclidean** | `√((r − rg)² + (c − cg)²)` | Straight-line distance, a weaker estimate |
| **Chebyshev** | `max(\|r − rg\|, \|c − cg\|)` | Suited to 8-direction movement, the weakest here |

None of the three overestimates the true cost, so all of them are **admissible** and A\* returns a shortest path with each. They differ only in how much of the grid they have to search.

### Plain A\* for many robots

Each robot runs its own A\*. A simulation then moves everyone and checks for conflicts. When two robots are about to clash, one waits (chosen by a fixed index rule) and re-plans. This is **reactive**: conflicts are found after the paths are made, and re-plans pile up as robots increase.

### DPTA\* (our method)

DPTA\* plans robots **one after another over cells *and* time**, so conflicts are avoided before anyone moves.

| # | Idea | What it does |
| --- | --- | --- |
| 1 | **Dynamic priority** | `priority = distance + 2 × congestion`. Long trips and robots in crowded areas (other starts/goals within 5 cells) are planned first. |
| 2 | **Reservation table + time-aware states** | Each search state is `(row, col, time)`. Planned cells are reserved at specific times, and swaps are forbidden. **Wait** is a fifth move. |
| 3 | **True-distance heuristic** | A backward **BFS from each goal** gives wall-aware distances, replacing Manhattan. |
| 4 | **Parked robots** | A robot at its goal keeps its cell reserved for all future times. |
| 5 | **Priority repair** | If a robot fails to find a path, it is moved to the front of the order and everyone is re-planned (max 7 attempts). |

| Point | Plain A\* | DPTA\* |
| --- | --- | --- |
| Planning style | Reactive | Proactive |
| Vertex / swap conflicts | Found during execution | Prevented by the reservation table |
| Waiting | Limited | A proper wait move |
| Robot at its goal | Can block others | Parked and reserved |
| Heuristic | Manhattan | True BFS distance |
| Crowded areas | Not considered | Used in the priority score |
| A robot that fails | Local re-plan | Priority repair |
| Time | Mostly ignored | Part of the state |

**Trade-off:** DPTA\* does more work at planning time (BFS per goal, reservation checks, search over time, possible retries). The payoff comes later as fewer conflicts, fewer re-plans and less waiting.

---

## Experimental Setup

### Problem 1

- For each trial, a new random grid is generated: **30 × 30, 25% obstacle probability per cell**, with a random free start and a different random free goal.
- A\* is run once per heuristic **on the same grid**, so the comparison is fair.
- **100 trials**; every run is saved as a row in a CSV file, and a separate analysis script averages the rows and draws the graphs.

| Metric | Meaning |
| --- | --- |
| Nodes expanded | Cells taken out of the priority queue (machine-independent measure of search effort) |
| Execution time | Time for one A\* run |
| Path length | Number of moves from start to goal |
| Success rate | Share of grids where a path was found |

### Problem 2

Plain A\* and DPTA\* are run on the **same grids, starts and goals**, while varying the obstacle density, the grid size and the number of robots.

| Configuration | Grid size | Obstacles | Robots | Trials |
| --- | --- | --- | --- | --- |
| Reported run | 30 × 30 | 25% | 20 | _TODO: fill in_ |

Metrics: robots reaching goal, average path length, average arrival time, makespan, predicted conflicts, vertex conflicts, edge conflicts, re-plans, nodes expanded, planning time, total runtime and deadlocks.

---

## Results

### Problem 1: Heuristic comparison

30 × 30 grid, 25% obstacles, 100 random trials.

| Heuristic | Avg. nodes expanded | Avg. execution time (s) | Avg. path length | Success rate (%) |
| --- | --- | --- | --- | --- |
| **Manhattan** | **94.66** | **0.000120** | 23.08 | 96.0 |
| Euclidean | 127.83 | 0.000161 | 23.08 | 96.0 |
| Chebyshev | 151.23 | 0.000177 | 23.08 | 96.0 |

**Takeaways**

- All three heuristics return the **same path length** and success rate (about 4 in 100 grids had no path), as expected for admissible heuristics.
- **Manhattan is the best** for 4-direction grids: about 26% fewer nodes than Euclidean and 37% fewer than Chebyshev, and it is also the fastest.
- A closer estimate of the true distance lets A\* ignore more cells that point away from the goal.

### Problem 2: Plain A\* vs DPTA\*

30 × 30 grid, 25% obstacles, 20 robots. Values are approximate, read from the comparison graphs.

| Metric | Plain A\* | DPTA\* |
| --- | --- | --- |
| Vertex conflicts | ≈ 5 | **0** |
| Edge conflicts | ≈ 2 | **0** |
| Re-plans | ≈ 5 | **0** |
| Makespan (time steps) | ≈ 61 | **≈ 46** |
| Avg. path length | ≈ 23–24 | **≈ 21** |
| Planning time | ≈ 111 ms | ≈ 401 ms |
| Total runtime | ≈ 127 ms | ≈ 403 ms |
| Nodes expanded | ≈ 54,500 | ≈ 85,000 |

**Takeaways**

- DPTA\* removed **all vertex and edge conflicts and all re-plans**, and cut the makespan by about 25%.
- The cost is about **4× the planning time** and **1.5× the nodes expanded**. DPTA\* spends more computation up front for smoother, faster robot motion.

<!-- TODO: replace the approximate values above with exact numbers from your results CSV. -->

---

## Installation

**Requirements:** Python 3.8 or newer (developed on Python 3.13, macOS).

```bash
# 1. Clone the repository
git clone <your-repository-url>
cd <repository-folder>

# 2. (Optional) create a virtual environment
python3 -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate

# 3. Install dependencies
pip install pygame matplotlib
```

> If any script imports `pandas` or `numpy` directly, add them to the `pip install` line and to `requirements.txt`.

---

## Usage

Run the scripts from inside the problem folder, since they import each other locally.

### Problem 1: Single robot (`PS1/`)

```bash
cd PS1
python3 main.py        # Pygame simulator: pick heuristic, grid size, obstacle %
python3 exp.py         # runs the 100 random trials -> results.csv
python3 analysis.py    # averages results.csv -> summary.csv
python3 plots.py       # draws the comparison graphs
```

The plots are saved as:

| File | Shows |
| --- | --- |
| `average_nodes_expansion.png` | Average nodes expanded per heuristic |
| `average_execution_time.png` | Average execution time per heuristic |
| `average_path_length.png` | Average path length per heuristic |

### Problem 2: Multiple robots (`PS2/`)

```bash
cd PS2
python3 multi_main.py          # Pygame multi-robot simulator
python3 benchmark.py           # runs plain A* vs DPTA* -> robot_results.csv
python3 plot_robot_results.py  # reads robot_results.csv -> the four comparison PNGs
```

The plots are saved as:

| File | Shows |
| --- | --- |
| `01_path_execution_comparison.png` | Path length, arrival time and makespan |
| `02_conflict_comparison.png` | Vertex / edge conflicts and re-plans |
| `03_computational_cost_comparison.png` | Planning time and total runtime |
| `04_nodes_expanded_comparison.png` | Search effort (nodes expanded) |

---

## Project Structure

```
.
├── PS1/                              # Problem 1: single robot, 3 heuristics
│   ├── grid.py                       # random grid, start and goal generation
│   ├── astar.py                      # A* with Manhattan / Euclidean / Chebyshev heuristics
│   ├── main.py                       # Pygame simulator (entry point)
│   ├── exp.py                        # runs the 100-trial experiment
│   ├── results.py                    # collects per-trial metrics and writes results.csv
│   ├── results.csv                   # raw results, one row per run
│   ├── analysis.py                   # averages the raw results
│   ├── summary.csv                   # averaged results per heuristic
│   ├── plots.py                      # draws the comparison graphs
│   ├── average_nodes_expansion.png
│   ├── average_execution_time.png
│   └── average_path_length.png
├── PS2/                              # Problem 2: multi-robot path finding
│   ├── astar.py                      # A* search (plain A*, baseline)
│   ├── cooperative_astar.py          # DPTA*: time-aware, reservation-table planner
│   ├── multigrid.py                  # grid, obstacle, start/goal generation for many robots
│   ├── simulation.py                 # executes plans, detects conflicts, handles re-plans
│   ├── multi_main.py                 # Pygame multi-robot simulator (entry point)
│   ├── benchmark.py                  # runs the experiments, writes robot_results.csv
│   ├── robot_results.csv             # raw experiment results
│   ├── plot_robot_results.py         # draws the comparison graphs
│   ├── 01_path_execution_comparison.png
│   ├── 02_conflict_comparison.png
│   ├── 03_computational_cost_comparison.png
│   └── 04_nodes_expanded_comparison.png
├── .gitignore                        # should ignore venv/ and __pycache__/
├── requirements.txt
└── README.md
```

---

## Conclusion

- **Problem 1:** All three heuristics are admissible and give identical optimal paths. **Manhattan** expands the fewest nodes and runs fastest on 4-direction grids.
- **Problem 2:** Independent A\* paths clash. **DPTA\*** plans proactively with dynamic priorities, a time-aware reservation table, a BFS true-distance heuristic, parked-robot handling and priority repair. It achieved zero conflicts and zero re-plans with a shorter makespan, at the price of more planning computation.

**Possible future work:** diagonal movement, moving obstacles, larger robot counts and comparison with optimal methods such as Conflict-Based Search.

---

## References

1. Hart, P. E., Nilsson, N. J., and Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.
2. Silver, D. (2005). Cooperative pathfinding. *Proceedings of the 1st AIIDE*, 117–122.
3. Standley, T. (2010). Finding optimal solutions to cooperative pathfinding problems. *Proceedings of the AAAI Conference on Artificial Intelligence*, 173–178.
4. Russell, S., and Norvig, P. (2021). *Artificial Intelligence: A Modern Approach* (4th ed.). Pearson.
