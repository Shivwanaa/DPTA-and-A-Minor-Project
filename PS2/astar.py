import heapq
import time
import math
def manhattan(r, c, gr, gc):
    return abs(r - gr) + abs(c - gc)

def euclidean(r, c, gr, gc):
    return math.sqrt((r - gr) ** 2 + (c - gc) ** 2)

def chebyshev(r, c, gr, gc):
    return max(abs(r - gr), abs(c - gc))

def astar(grid, start, goal, heuristic, blocked=None):
    start_time = time.perf_counter()
    rows, cols = len(grid), len(grid[0])
    sr, sc = start
    gr, gc = goal
    pq = []
    g_cost = [[float('inf')] * cols for _ in range(rows)]
    parent = [[None] * cols for _ in range(rows)]
    explored = []
    heapq.heappush(pq, (heuristic(sr, sc, gr, gc), 0, sr, sc))
    g_cost[sr][sc] = 0
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
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
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue
            if grid[nr][nc] == 1:
                continue
            if blocked and (nr, nc) in blocked:
                continue
            new_g = g + 1
            if new_g < g_cost[nr][nc]:
                g_cost[nr][nc] = new_g
                parent[nr][nc] = (r, c)
                heapq.heappush(
                    pq, (new_g + heuristic(nr, nc, gr, gc), new_g, nr, nc))
    execution_time = time.perf_counter() - start_time
    if g_cost[gr][gc] == float('inf'):
        return {"path": None, "explored": explored,
                "nodes_expanded": nodes_expanded,
                "time": execution_time, "path_length": None}

    path, current = [], (gr, gc)
    while current is not None:
        path.append(current)
        current = parent[current[0]][current[1]]
    path.reverse()
    return {"path": path, "explored": explored,
            "nodes_expanded": nodes_expanded,
            "time": execution_time, "path_length": len(path) - 1}

def multi_robot_astar(grid, starts, goals, heuristic):
    """Plain A*: every robot plans on its own, ignoring the others."""
    return [astar(grid, starts[i], goals[i], heuristic)
            for i in range(len(starts))]