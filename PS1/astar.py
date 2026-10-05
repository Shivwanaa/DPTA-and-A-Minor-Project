import heapq
import time
import math

#manhattan heuristic
def manhattan(r, c, gr, gc):
    return abs(r - gr) + abs(c - gc)

#euclidean heuristic
def euclidean(r, c, gr, gc):
    return math.sqrt(
        (r - gr) ** 2 +
        (c - gc) ** 2
    )

#chebyshev heuristic
def chebyshev(r, c, gr, gc):

    return max(
        abs(r - gr),
        abs(c - gc)
    )

#A* algo
def astar(grid, start, goal, heuristic):
    start_time = time.perf_counter()
    rows = len(grid)
    cols = len(grid[0])
    sr, sc = start
    gr, gc = goal
    pq = []
    g_cost = [
        [float('inf')] * cols
        for _ in range(rows)
    ]
    parent = [
        [None] * cols
        for _ in range(rows)
    ]
    explored = []
    h = heuristic(sr, sc, gr, gc)
    heapq.heappush(
        pq,
        (h, 0, sr, sc)
    )
    g_cost[sr][sc] = 0
    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]
    nodes_expanded = 0
    while pq:
        f, g, r, c = heapq.heappop(pq)
        if g != g_cost[r][c]:
            continue
        nodes_expanded += 1
        explored.append((r, c))
        if (r, c) == (gr, gc):
            break
        for dr, dc in directions:
            nr = r + dr
            nc = c + dc
            if nr < 0 or nr >= rows:
                continue
            if nc < 0 or nc >= cols:
                continue
            if grid[nr][nc] == 1:
                continue
            new_g = g + 1
            if new_g < g_cost[nr][nc]:
                g_cost[nr][nc] = new_g
                h = heuristic(
                    nr,
                    nc,
                    gr,
                    gc
                )
                new_f = new_g + h
                heapq.heappush(
                    pq,
                    (
                        new_f,
                        new_g,
                        nr,
                        nc
                    )
                )
                parent[nr][nc] = (r, c)
    execution_time = (
        time.perf_counter() - start_time
    )
    if g_cost[gr][gc] == float('inf'):
        return {
            "path": None,
            "explored": explored,
            "nodes_expanded": nodes_expanded,
            "time": execution_time,
            "path_length": None
        }
    path = []
    current = (gr, gc)
    while current is not None:
        path.append(current)
        r, c = current
        current = parent[r][c]
    path.reverse()
    path_length = len(path) - 1
    return {
        "path": path,
        "explored": explored,
        "nodes_expanded": nodes_expanded,
        "time": execution_time,
        "path_length": path_length
    }

def multi_robot_astar(
    grid,
    starts,
    goals,
    heuristic
):
    results = []
    for i in range(len(starts)):
        start = starts[i]
        goal = goals[i]
        result = astar(
            grid,
            start,
            goal,
            heuristic
        )
        results.append(result)

    return results