import time
from astar import astar, multi_robot_astar, manhattan
from cooperative_astar import dpta_star

def find_conflicts(pos, nxt):
    out = []
    n = len(pos)
    for i in range(n):
        for j in range(i + 1, n):
            if nxt[i] == nxt[j]:
                out.append({"kind": "vertex", "robots": (i, j),
                            "cells": [nxt[i]]})
            elif nxt[i] == pos[j] and nxt[j] == pos[i]:
                out.append({"kind": "edge", "robots": (i, j),
                            "cells": [pos[i], pos[j]]})
    return out

def _key(c):
    return (c["kind"], c["robots"], tuple(c["cells"]))

def _new_events(conflicts, prev_keys):
    """Keep only conflicts that were not already present in the last tick."""
    keys = {_key(c) for c in conflicts}
    return [c for c in conflicts if _key(c) not in prev_keys], keys

def count_frames(frames, upto=None):
    """(vertex, edge, replans) accumulated over frames 1..upto."""
    upto = len(frames) - 1 if upto is None else upto
    v = e = r = 0
    for fr in frames[1:upto + 1]:
        v += sum(c["kind"] == "vertex" for c in fr["events"])
        e += sum(c["kind"] == "edge" for c in fr["events"])
        r += len(fr["replanned"])
    return v, e, r

def _frame(pos, events, replanned, paths):
    return {"pos": list(pos), "events": events, "replanned": replanned,
            "paths": [list(p) if p else None for p in paths]}

def _loser(i, j, pos, nxt):
    """Who gives way. A robot that stands still always wins."""
    i_stay, j_stay = nxt[i] == pos[i], nxt[j] == pos[j]
    if i_stay and not j_stay:
        return j
    if j_stay and not i_stay:
        return i
    return max(i, j)                       

def _text(c, tick):
    i, j = c["robots"]
    r, cc = c["cells"][0]
    kind = "edge conflict (swap)" if c["kind"] == "edge" else "vertex conflict"
    return f"t={tick}: {kind} R{i+1} vs R{j+1} at ({r},{cc})"

def simulate_planned(starts, paths):
    n = len(starts)
    def at(i, t):
        p = paths[i]
        return starts[i] if p is None else p[min(t, len(p) - 1)]

    T = max([len(p) - 1 for p in paths if p] + [0])
    frames = [_frame(starts, [], [], paths)]
    log, prev_keys = [], set()

    for t in range(1, T + 1):
        prev = [at(i, t - 1) for i in range(n)]
        cur = [at(i, t) for i in range(n)]
        events, prev_keys = _new_events(find_conflicts(prev, cur), prev_keys)
        log += [(t, _text(e, t)) for e in events]
        rem = [p[min(t, len(p) - 1):] if p else None for p in paths]
        frames.append(_frame(cur, events, [], rem))

    arrival = [len(p) - 1 if p else None for p in paths]
    return {"frames": frames, "arrival": arrival, "deadlock": False,
            "log": log, "extra_nodes": 0, "extra_time": 0.0}

def simulate_plain(grid, starts, goals, initial_paths,
                   heuristic=manhattan, stall_limit=15, max_ticks=None):
    n = len(starts)
    rows, cols = len(grid), len(grid[0])
    if max_ticks is None:
        max_ticks = 4 * (rows + cols) + 40
    pos = list(starts)
    paths = [list(p) if p else None for p in initial_paths]
    arrival = [None] * n
    frames = [_frame(pos, [], [], paths)]
    extra_nodes = stalled = 0
    extra_time, log, deadlock = 0.0, [], False
    prev_keys, prev_failed = set(), set()
    for tick in range(1, max_ticks + 1):
        if all(arrival[i] is not None or paths[i] is None for i in range(n)):
            break
        nxt = [paths[i][1] if paths[i] and len(paths[i]) > 1 else pos[i]
               for i in range(n)]
        conflicts = find_conflicts(pos, nxt)
        events, prev_keys = _new_events(conflicts, prev_keys)
        log += [(tick, _text(e, tick)) for e in events]
        losers = sorted({_loser(c["robots"][0], c["robots"][1], pos, nxt)
                         for c in conflicts})
        conf = find_conflicts(pos, nxt)
        while conf:
            i, j = conf[0]["robots"]
            l = _loser(i, j, pos, nxt)
            nxt[l] = pos[l]
            conf = find_conflicts(pos, nxt)
        replanned, failed = [], set()
        for l in losers:
            blocked = {pos[k] for k in range(n) if k != l}
            blocked |= {nxt[k] for k in range(n) if k != l}
            blocked.discard(pos[l])
            res = astar(grid, pos[l], goals[l], heuristic, blocked)
            extra_nodes += res["nodes_expanded"]
            extra_time += res["time"]
            if res["path"] and len(res["path"]) > 1:
                paths[l] = res["path"]
                nxt[l] = res["path"][1]
                replanned.append(l)
                log.append((tick, f"t={tick}: R{l+1} re-plans, new path "
                                  f"({res['path_length']} steps)"))
            else:
                failed.add(l)
                if l not in prev_failed:
                    log.append((tick, f"t={tick}: R{l+1} has no way around, "
                                      f"waits"))
        prev_failed = failed
        moved = False
        for i in range(n):
            if nxt[i] != pos[i]:
                pos[i] = nxt[i]
                paths[i] = paths[i][1:]
                moved = True
                if pos[i] == goals[i]:
                    arrival[i] = tick
        frames.append(_frame(pos, events, replanned, paths))
        stalled = 0 if moved else stalled + 1
        if stalled >= stall_limit:
            deadlock = True
            log.append((tick, f"t={tick}: DEADLOCK - no robot can move"))
            break
    return {"frames": frames, "arrival": arrival, "deadlock": deadlock,
            "log": log, "extra_nodes": extra_nodes, "extra_time": extra_time}

def _metrics(sim, n, nodes, plan_time, runtime, predicted):
    frames = sim["frames"]
    v, e, r = count_frames(frames)
    pv, pe, _ = count_frames(predicted["frames"])
    moves = [0] * n
    for a, b in zip(frames, frames[1:]):
        for i in range(n):
            if a["pos"][i] != b["pos"][i]:
                moves[i] += 1
    ok = [i for i in range(n) if sim["arrival"][i] is not None]
    arr = [sim["arrival"][i] for i in ok]
    return {
        "success": len(ok), "total": n,
        "success_rate": 100.0 * len(ok) / n,
        "avg_path_length": sum(moves[i] for i in ok) / len(ok) if ok else 0,
        "avg_arrival": sum(arr) / len(arr) if arr else 0,
        "makespan": max(arr) if arr else 0,
        "predicted": pv + pe,
        "vertex": v, "edge": e, "collisions": v + e, "replans": r,
        "nodes": nodes,
        "plan_time_ms": plan_time * 1000,
        "runtime_ms": runtime * 1000,
        "deadlock": sim["deadlock"],
    }

def run_plain(grid, starts, goals, heuristic=manhattan):
    t0 = time.perf_counter()
    results = multi_robot_astar(grid, starts, goals, heuristic)
    paths = [r["path"] for r in results]
    predicted = simulate_planned(starts, paths)
    sim = simulate_plain(grid, starts, goals, paths, heuristic)
    nodes = sum(r["nodes_expanded"] for r in results) + sim["extra_nodes"]
    ptime = sum(r["time"] for r in results) + sim["extra_time"]
    runtime = time.perf_counter() - t0
    return {"plan_paths": paths, "predicted": predicted, "sim": sim,
            "metrics": _metrics(sim, len(starts), nodes, ptime, runtime,
                                predicted)}

def run_dpta(grid, starts, goals):
    t0 = time.perf_counter()
    res = dpta_star(grid, starts, goals)
    paths = res["paths"]
    sim = simulate_planned(starts, paths)
    runtime = time.perf_counter() - t0
    return {"plan_paths": paths, "predicted": sim, "sim": sim,
            "metrics": _metrics(sim, len(starts), res["total_nodes_expanded"],
                                res["total_time"], runtime, sim)}