import random
from collections import deque

def _components(grid):
    rows, cols = len(grid), len(grid[0])
    seen = [[False] * cols for _ in range(rows)]
    comps = []
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 0 and not seen[r][c]:
                comp, q = [], deque([(r, c)])
                seen[r][c] = True
                while q:
                    cr, cc = q.popleft()
                    comp.append((cr, cc))
                    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nr, nc = cr + dr, cc + dc
                        if (0 <= nr < rows and 0 <= nc < cols
                                and grid[nr][nc] == 0 and not seen[nr][nc]):
                            seen[nr][nc] = True
                            q.append((nr, nc))
                comps.append(comp)
    return comps

def create_multi_robot_grid(rows, cols, obstacle_probability, num_robots,
                            seed=None, attempts=30):
    rng = random.Random(seed)
    for _ in range(attempts):
        grid = [[1 if rng.random() < obstacle_probability else 0
                 for _ in range(cols)] for _ in range(rows)]

        comps = [c for c in _components(grid) if len(c) >= 2]
        pools = [rng.sample(c, len(c)) for c in comps]

        starts, goals, ok = [], [], True
        for _ in range(num_robots):
            options = [p for p in pools if len(p) >= 2]
            if not options:
                ok = False
                break
            pool = rng.choices(options, weights=[len(p) for p in options])[0]
            starts.append(pool.pop())
            goals.append(pool.pop())

        if ok:
            return grid, starts, goals

    return None, None, None

def print_multi_grid(grid, starts, goals):
    smap = {s: i + 1 for i, s in enumerate(starts)}
    gmap = {g: i + 1 for i, g in enumerate(goals)}
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if (r, c) in smap:
                print(f"S{smap[(r, c)]}", end=" ")
            elif (r, c) in gmap:
                print(f"G{gmap[(r, c)]}", end=" ")
            elif grid[r][c] == 1:
                print("#", end=" ")
            else:
                print(".", end=" ")
        print()
