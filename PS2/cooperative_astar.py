"""
DPTA*  -  Dynamic-Priority Time-aware A*   (our proposed method)

Ideas
 1. Prioritised planning: robots plan one after another; every finished
    plan is written into a space-time RESERVATION TABLE, so later robots
    plan around it (they may WAIT or detour) -> conflicts are avoided at
    planning time instead of being repaired at execution time.
 2. Congestion-aware priority: long trips and robots in crowded areas plan
    first (priority = distance + 2 * nearby starts/goals).
 3. Parked robots: a robot sitting on its goal blocks that cell forever.
 4. Priority repair: if a robot cannot find a path, it is promoted to the
    front of the order and everybody is re-planned.
 5. Exact-distance heuristic (BFS from the goal) for the space-time search.
"""
import heapq
import time
from collections import deque
from astar import manhattan

DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]   

def calculate_priority(i, starts, goals, radius=5):
    sr, sc = starts[i]
    gr, gc = goals[i]
    distance = manhattan(sr, sc, gr, gc)

    congestion = 0
    for j in range(len(starts)):
        if j == i:
            continue
        if manhattan(sr, sc, *starts[j]) <= radius:
            congestion += 1
        if manhattan(gr, gc, *goals[j]) <= radius:
            congestion += 1

    return distance + 2 * congestion

def bfs_distance(grid, goal):
    """True shortest distance from every free cell to `goal`."""
    rows, cols = len(grid), len(grid[0])
    dist = {goal: 0}
    q = deque([goal])
    while q:
        r, c = q.popleft()
        for dr, dc in DIRS[:4]:
            nr, nc = r + dr, c + dc
            if (0 <= nr < rows and 0 <= nc < cols
                    and grid[nr][nc] == 0 and (nr, nc) not in dist):
                dist[(nr, nc)] = dist[(r, c)] + 1
                q.append((nr, nc))
    return dist

class Reservations:
    def __init__(self):
        self.vertex = set()      
        self.edge = set()        
        self.parked = {}         
        self.last_time = {}

    def vertex_blocked(self, cell, t):
        if (cell, t) in self.vertex:
            return True
        return cell in self.parked and t >= self.parked[cell]

    def edge_blocked(self, cur, nxt, t):
        return (nxt, cur, t) in self.edge

    def can_park(self, cell, t):
        return self.last_time.get(cell, -1) < t

    def reserve(self, path):
        for t, cell in enumerate(path):
            self.vertex.add((cell, t))
            self.last_time[cell] = max(self.last_time.get(cell, -1), t)
        for t in range(len(path) - 1):
            if path[t] != path[t + 1]:
                self.edge.add((path[t], path[t + 1], t))
        self.parked[path[-1]] = len(path) - 1

def time_aware_astar(grid, start, goal, res, dist,
                     max_time, max_nodes=150000):
    t_start = time.perf_counter()
    def fail(nodes):
        return {"path": None, "nodes_expanded": nodes, "success": False,
                "time": time.perf_counter() - t_start, "path_length": None}
    if start not in dist:
        return fail(0)
    pq = [(dist[start], 0, start[0], start[1])]     
    seen = {(start[0], start[1], 0)}
    parent = {}
    nodes = 0
    while pq:
        f, neg_t, r, c = heapq.heappop(pq)
        t = -neg_t
        nodes += 1
        if (r, c) == goal and res.can_park(goal, t):
            path, state = [], (r, c, t)
            while state is not None:
                path.append((state[0], state[1]))
                state = parent.get(state)
            path.reverse()
            return {"path": path, "nodes_expanded": nodes, "success": True,
                    "time": time.perf_counter() - t_start,
                    "path_length": len(path) - 1}

        if nodes > max_nodes:
            break
        if t >= max_time:
            continue
        for dr, dc in DIRS:
            nr, nc, nt = r + dr, c + dc, t + 1
            if (nr, nc) not in dist:                 
                continue
            if res.vertex_blocked((nr, nc), nt):
                continue
            if (dr or dc) and res.edge_blocked((r, c), (nr, nc), t):
                continue
            state = (nr, nc, nt)
            if state in seen:
                continue
            seen.add(state)
            parent[state] = (r, c, t)
            heapq.heappush(pq, (nt + dist[(nr, nc)], -nt, nr, nc))
    return fail(nodes)

def dpta_star(grid, starts, goals, max_retries=6):
    t0 = time.perf_counter()
    n = len(starts)
    prio = [calculate_priority(i, starts, goals) for i in range(n)]
    order = sorted(range(n), key=lambda i: (-prio[i], i))
    dists = [bfs_distance(grid, goals[i]) for i in range(n)]
    best, total_nodes, attempts = None, 0, 0
    for _ in range(max_retries + 1):
        attempts += 1
        res = Reservations()
        results = [None] * n
        failed = []
        for i in order:
            slack = 4 * n + 20
            max_time = dists[i].get(starts[i], 0) + slack
            r = time_aware_astar(grid, starts[i], goals[i], res,
                                 dists[i], max_time)
            r["priority"] = prio[i]
            results[i] = r
            total_nodes += r["nodes_expanded"]
            if r["success"]:
                res.reserve(r["path"])
            else:
                failed.append(i)
        ok = n - len(failed)
        if best is None or ok > best[0]:
            best = (ok, results)
        if not failed:
            break
        cand = [i for i in failed
                if order.index(i) > 0 and starts[i] in dists[i]]
        if not cand:
            break
        order.remove(cand[0])
        order.insert(0, cand[0])

    ok, results = best
    lengths = [r["path_length"] for r in results if r["success"]]
    return {
        "robots": results,
        "paths": [r["path"] for r in results],
        "successful": ok,
        "total_robots": n,
        "average_path_length": sum(lengths) / len(lengths) if lengths else None,
        "total_nodes_expanded": total_nodes,
        "total_time": time.perf_counter() - t0,
        "attempts": attempts,
    }
