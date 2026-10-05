import csv
import pygame
from multigrid import create_multi_robot_grid
from simulation import run_plain, run_dpta, count_frames

WIDTH = 1120
HEIGHT = 720
GRID_SIZE = 600
PANEL_X = 620
TICK_DELAY = 300       
WHITE = (255, 255, 255)
BLACK = (30, 30, 30)
GRAY = (180, 180, 180)
DARK = (50, 50, 50)
BLUE = (80, 150, 255)
GREEN = (0, 200, 0)
RED = (220, 0, 0)
ORANGE = (240, 150, 30)
TEAL = (0, 160, 160)
ROBOT_COLORS = [
    (50, 120, 220), (50, 180, 80), (240, 180, 50), (160, 80, 200),
    (230, 100, 50), (50, 180, 180), (220, 80, 140), (100, 100, 220),
    (120, 180, 80), (200, 120, 200), (80, 160, 200), (200, 160, 80),
    (100, 200, 160), (180, 100, 100), (120, 120, 180), (180, 180, 80),
    (80, 180, 180), (200, 100, 160), (140, 180, 100), (180, 120, 60)
]
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Multi-Robot Path Finding - Plain A* vs DPTA*")
clock = pygame.time.Clock()

font = pygame.font.SysFont(None, 25)
small_font = pygame.font.SysFont(None, 20)
ROWS, COLS, CELL_SIZE = 20, 20, 30
grid, starts, goals = [], [], []
results = {}                 
method = None
started = False              
playing = False
tick = 0
last_tick_time = 0

trails, marks, replan_tags = [], [], {}
message = ""
METHOD_NAMES = {
    "plain": "Plain A*",
    "dpta": "DPTA*"
}
inputs = {
    "rows": "20",
    "cols": "20",
    "obstacle": "25",
    "robots": "6"
}
labels = {
    "rows": "Rows:",
    "cols": "Columns:",
    "obstacle": "Obstacle %:",
    "robots": "Robots:"
}
max_len = {
    "rows": 2,
    "cols": 2,
    "obstacle": 2,
    "robots": 2
}
boxes = {
    key: pygame.Rect(PANEL_X + 120, 52 + 36 * i, 100, 28)
    for i, key in enumerate(inputs)
}
active_input = None
fresh = False
def make_button(col, row, text):
    return text, pygame.Rect(
        PANEL_X + col * 245,
        205 + row * 45,
        235,
        38
    )
BUTTONS = {
    "generate": make_button(0, 0, "Generate Grid"),
    "reset": make_button(1, 0, "Reset"),
    "plain": make_button(0, 1, "Plain A*"),
    "dpta": make_button(1, 1, "DPTA*"),
    "execute": make_button(0, 2, "Execute / Pause"),
    "both": make_button(1, 2, "Compare Both"),
    "csv": make_button(0, 3, "Export CSV"),
}
TABLE = [
    ("Robots reaching goal", "success", "hi"),
    ("Avg path length", "avg_path_length", "lo"),
    ("Avg arrival time", "avg_arrival", "lo"),
    ("Makespan (ticks)", "makespan", "lo"),
    ("Predicted conflicts", "predicted", "lo"),
    ("Vertex conflicts", "vertex", "lo"),
    ("Edge conflicts", "edge", "lo"),
    ("Re-plans", "replans", "lo"),
    ("Nodes expanded", "nodes", "lo"),
    ("Planning time (ms)", "plan_time_ms", "lo"),
    ("Total run time (ms)", "runtime_ms", "lo"),
    ("Deadlock", "deadlock", "lo"),
]

def fmt(key, m):
    v = m[key]
    if key == "success":
        return f"{m['success']}/{m['total']}"
    if key == "deadlock":
        return "YES" if v else "no"
    if isinstance(v, float):
        return f"{v:.2f}"
    return str(v)

def reset_view():
    global started, playing, tick, trails, marks, replan_tags

    started = False
    playing = False
    tick = 0

    trails = [[s] for s in starts]
    marks = []
    replan_tags = {}


def generate_grid():
    global grid, starts, goals, ROWS, COLS, CELL_SIZE
    global results, method, message
    try:
        rows = max(5, min(50, int(inputs["rows"])))
        cols = max(5, min(50, int(inputs["cols"])))
        obstacle = max(0, min(80, int(inputs["obstacle"])))
        robots = max(1, min(20, int(inputs["robots"])))
    except ValueError:
        message = "Please fill in all four boxes with numbers"
        return
    new_grid, new_starts, new_goals = create_multi_robot_grid(
        rows,
        cols,
        obstacle / 100,
        robots
    )
    if new_grid is None:
        message = "Not enough free cells - lower obstacles or robots"
        return
    inputs.update(
        rows=str(rows),
        cols=str(cols),
        obstacle=str(obstacle),
        robots=str(robots)
    )
    grid = new_grid
    starts = new_starts
    goals = new_goals
    ROWS = rows
    COLS = cols
    CELL_SIZE = min(
        GRID_SIZE // COLS,
        GRID_SIZE // ROWS
    )
    results = {}
    method = None
    reset_view()
    message = "New grid generated"


def show_message(text_message):
    global message
    message = text_message
    draw()
    pygame.display.flip()


def plan(key):
    global method, message
    if key not in results:
        show_message(
            f"Planning with {METHOD_NAMES[key]} ..."
        )
        run = run_plain if key == "plain" else run_dpta
        results[key] = run(grid, starts, goals)

    method = key
    reset_view()
    m = results[key]["metrics"]
    message = (
        f"{METHOD_NAMES[key]}: plan phase shows "
        f"{m['predicted']} predicted conflict(s)"
    )

def compare_both():
    global message
    plan("plain")
    plan("dpta")
    plan("plain")
    message = (
        "Both planners done - press Execute, "
        "switch planner to compare"
    )

def execute():
    global started, playing, tick, last_tick_time
    global trails, marks, replan_tags, message
    if method is None:
        message = "Choose Plain A* or DPTA* first"
        return
    frames = results[method]["sim"]["frames"]
    if not started or tick >= len(frames) - 1:
        started = True
        playing = True
        tick = 0
        trails = [[s] for s in starts]
        marks = []
        replan_tags = {}
        last_tick_time = pygame.time.get_ticks()
        message = "Execution started"
    else:
        playing = not playing

def export_csv():
    global message
    if not results:
        message = "Run a planner first"
        return
    with open("robot_results.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "rows",
            ROWS,
            "cols",
            COLS,
            "robots",
            len(starts)
        ])
        w.writerow([
            "metric",
            "Plain A*",
            "DPTA*"
        ])
        for label, key, _ in TABLE:
            w.writerow([
                label
            ] + [
                fmt(key, results[m]["metrics"])
                if m in results
                else ""
                for m in ("plain", "dpta")
            ])

    message = "Saved robot_results.csv"

def advance():
    global tick, playing, message
    frames = results[method]["sim"]["frames"]
    if tick >= len(frames) - 1:
        playing = False
        return
    tick += 1
    frame = frames[tick]
    for i, p in enumerate(frame["pos"]):
        if trails[i][-1] != p:
            trails[i].append(p)
    for e in frame["events"]:
        for cell in e["cells"]:
            marks.append(
                (cell, e["kind"])
            )
    for i in frame["replanned"]:
        replan_tags[i] = tick
    if tick >= len(frames) - 1:
        playing = False
        sim = results[method]["sim"]
        if sim["deadlock"]:
            message = (
                "Finished - DEADLOCK, "
                "some robots are stuck"
            )
        else:
            message = (
                "Finished - all robots at their goals"
            )

def cell_rect(cell, inset=0):
    r, c = cell
    return pygame.Rect(
        c * CELL_SIZE + inset,
        r * CELL_SIZE + inset,
        CELL_SIZE - 2 * inset,
        CELL_SIZE - 2 * inset
    )

def center(cell, off=0):
    r, c = cell
    return (
        c * CELL_SIZE + CELL_SIZE // 2 + off,
        r * CELL_SIZE + CELL_SIZE // 2 + off
    )

def cell_text(text_value, cell, color=WHITE, scale=0.55):
    f = pygame.font.SysFont(
        None,
        max(12, int(CELL_SIZE * scale) + 6)
    )
    s = f.render(
        text_value,
        True,
        color
    )
    screen.blit(
        s,
        s.get_rect(center=center(cell))
    )

def draw_grid():
    if not grid:
        return
    n = len(starts)
    res = results.get(method)
    frame = (
        res["sim"]["frames"][tick]
        if (res and started)
        else None
    )
    for r in range(ROWS):
        for c in range(COLS):
            pygame.draw.rect(
                screen,
                DARK if grid[r][c] else WHITE,
                cell_rect((r, c))
            )
    for i in range(n):
        col = ROBOT_COLORS[i % 20]
        pygame.draw.rect(
            screen,
            col,
            cell_rect(goals[i], 2)
        )
        if CELL_SIZE >= 18:
            cell_text(
                f"G{i + 1}",
                goals[i]
            )
        pygame.draw.rect(
            screen,
            col,
            cell_rect(starts[i], 2),
            2
        )
    if frame:
        for i, trail in enumerate(trails):
            for cell in trail:
                pygame.draw.rect(
                    screen,
                    ROBOT_COLORS[i % 20],
                    cell_rect(
                        cell,
                        CELL_SIZE // 3
                    )
                )
    if res:
        paths = (
            frame["paths"]
            if frame
            else res["plan_paths"]
        )
        for i, p in enumerate(paths):
            if p and len(p) > 1:
                off = (
                    i % 5 - 2
                ) * max(
                    1,
                    CELL_SIZE // 10
                )
                pts = [
                    center(x, off)
                    for x in p
                ]
                pygame.draw.lines(
                    screen,
                    ROBOT_COLORS[i % 20],
                    False,
                    pts,
                    3 if not frame else 2
                )
    if res and not frame:
        for fr in res["predicted"]["frames"]:
            for e in fr["events"]:
                for cell in e["cells"]:
                    r = cell_rect(
                        cell,
                        2
                    )
                    pts = [
                        (r.centerx, r.top),
                        (r.right, r.bottom),
                        (r.left, r.bottom)
                    ]
                    pygame.draw.polygon(
                        screen,
                        ORANGE,
                        pts
                    )
                    pygame.draw.polygon(
                        screen,
                        BLACK,
                        pts,
                        2
                    )
    for cell, kind in marks:
        r = cell_rect(
            cell,
            1
        )
        if kind == "vertex":
            pygame.draw.rect(
                screen,
                (255, 200, 200),
                r
            )
            pygame.draw.line(
                screen,
                RED,
                r.topleft,
                r.bottomright,
                3
            )
            pygame.draw.line(
                screen,
                RED,
                r.topright,
                r.bottomleft,
                3
            )
        else:
            x = r.centerx
            y = r.centery
            d = r.width // 2
            pts = [
                (x, y - d),
                (x + d, y),
                (x, y + d),
                (x - d, y)
            ]
            pygame.draw.polygon(
                screen,
                (255, 225, 170),
                pts
            )
            pygame.draw.polygon(
                screen,
                ORANGE,
                pts,
                3
            )
    positions = (
        frame["pos"]
        if frame
        else starts
    )
    for i, cell in enumerate(positions):
        col = ROBOT_COLORS[i % 20]
        rad = int(
            CELL_SIZE * 0.38
        )
        pygame.draw.circle(
            screen,
            col,
            center(cell),
            rad
        )
        at_goal = (
            frame and
            cell == goals[i]
        )
        pygame.draw.circle(
            screen,
            WHITE if at_goal else BLACK,
            center(cell),
            rad,
            2
        )
        if CELL_SIZE >= 16:

            cell_text(
                str(i + 1),
                cell
            )
    if frame:
        for i, t in replan_tags.items():
            if tick - t <= 2:
                cx, cy = center(
                    frame["pos"][i]
                )
                s = max(
                    7,
                    int(CELL_SIZE * 0.45)
                )
                badge = pygame.Rect(
                    cx + CELL_SIZE // 2 - s,
                    cy - CELL_SIZE // 2,
                    s,
                    s
                )
                pygame.draw.rect(
                    screen,
                    TEAL,
                    badge
                )
                pygame.draw.rect(
                    screen,
                    WHITE,
                    badge,
                    1
                )
                if s >= 10:
                    f = pygame.font.SysFont(
                        None,
                        s + 4
                    )
                    txt = f.render(
                        "R",
                        True,
                        WHITE
                    )
                    screen.blit(
                        txt,
                        txt.get_rect(
                            center=badge.center
                        )
                    )
    for r in range(ROWS + 1):
        pygame.draw.line(
            screen,
            GRAY,
            (0, r * CELL_SIZE),
            (COLS * CELL_SIZE, r * CELL_SIZE)
        )
    for c in range(COLS + 1):
        pygame.draw.line(
            screen,
            GRAY,
            (c * CELL_SIZE, 0),
            (c * CELL_SIZE, ROWS * CELL_SIZE)
        )

def text(s, x, y, color=WHITE, f=small_font):

    screen.blit(
        f.render(s, True, color),
        (x, y)
    )

def draw_button(key):
    label, rect = BUTTONS[key]
    selected = (
        key == method or
        (key == "execute" and playing)
    )
    pygame.draw.rect(
        screen,
        BLUE if selected else GRAY,
        rect,
        border_radius=5
    )
    s = small_font.render(
        label,
        True,
        BLACK
    )
    screen.blit(
        s,
        s.get_rect(center=rect.center)
    )

def phase_name():
    if method is None:
        return "READY"
    if not started:
        return "PLAN"
    frames = results[method]["sim"]["frames"]
    if tick >= len(frames) - 1:
        return "DONE"
    return (
        "EXECUTING"
        if playing
        else "PAUSED"
    )

def draw_event_log():
    res = results.get(method)
    if not res:
        return
    x = 10
    y = GRID_SIZE + 8
    text(
        "Event Log",
        x,
        y,
        WHITE,
        font
    )
    y += 25
    if not started:
        src = res["predicted"]["log"]
        lines = [
            "plan: " + t
            for _, t in src[-4:]
        ]
    else:
        src = [
            e
            for e in res["sim"]["log"]
            if e[0] <= tick
        ]
        lines = [
            t
            for _, t in src[-4:]
        ]
    if not lines:
        text(
            "No events yet",
            x,
            y,
            GRAY
        )
        return
    for line in lines:
        text(
            line[:72],
            x,
            y,
            GRAY
        )
        y += 18

def draw_panel():
    text(
        "Multi-Robot A*",
        PANEL_X,
        15,
        WHITE,
        font
    )
    for key, box in boxes.items():
        text(
            labels[key],
            PANEL_X,
            box.y + 7
        )
        pygame.draw.rect(
            screen,
            WHITE,
            box
        )
        pygame.draw.rect(
            screen,
            BLUE if active_input == key else GRAY,
            box,
            2
        )
        shown = (
            inputs[key] +
            ("|" if active_input == key else "")
        )
        text(
            shown,
            box.x + 8,
            box.y + 6,
            BLACK
        )
    text(
        "(click a box, type, press Enter)",
        PANEL_X + 235,
        60,
        GRAY
    )
    for key in BUTTONS:
        draw_button(key)
    res = results.get(method)
    y = 395
    if res:
        frames = res["sim"]["frames"]
        v, e, r = count_frames(
            frames,
            tick if started else 0
        )
        text(
            f"Phase: {phase_name()}    "
            f"Method: {METHOD_NAMES[method]}    "
            f"Tick: {tick}/{len(frames) - 1}",
            PANEL_X,
            y
        )
        text(
            f"Conflicts so far:  "
            f"vertex {v}   "
            f"edge {e}   "
            f"total {v + e}   "
            f"re-plans {r}",
            PANEL_X,
            y + 20,
            ORANGE if v + e else WHITE
        )
    else:
        text(
            "Phase: READY",
            PANEL_X,
            y
        )
    text(
        message[:60],
        PANEL_X,
        y + 40,
        BLUE
    )
    y = 462
    text(
        "Final result (full execution)",
        PANEL_X,
        y,
        GRAY
    )
    text(
        "Plain A*",
        PANEL_X + 240,
        y,
        WHITE,
        small_font
    )
    text(
        "DPTA*",
        PANEL_X + 360,
        y,
        WHITE,
        small_font
    )
    both = (
        "plain" in results and
        "dpta" in results
    )
    for j, (label, key, better) in enumerate(TABLE):
        yy = y + 22 + j * 19
        text(
            label,
            PANEL_X,
            yy
        )
        win = None
        if both:
            a = float(
                results["plain"]["metrics"][key]
            )
            b = float(
                results["dpta"]["metrics"][key]
            )
            if a != b:
                win = (
                    "plain"
                    if (a > b) == (better == "hi")
                    else "dpta"
                )
        for m, x in (
            ("plain", PANEL_X + 240),
            ("dpta", PANEL_X + 360)
        ):
            if m not in results:
                text(
                    "-",
                    x,
                    yy,
                    GRAY
                )
            else:
                mt = results[m]["metrics"]
                col = (
                    GREEN
                    if win == m
                    else WHITE
                )
                if key == "deadlock" and mt[key]:
                    col = RED
                text(
                    fmt(key, mt),
                    x,
                    yy,
                    col
                )

def draw():
    screen.fill(BLACK)
    draw_grid()
    draw_event_log()
    draw_panel()

def click(pos):
    global active_input, fresh
    active_input = None
    for key, box in boxes.items():
        if box.collidepoint(pos):
            active_input = key
            fresh = True
    for key, (_, rect) in BUTTONS.items():
        if rect.collidepoint(pos):
            if key == "generate":
                generate_grid()
            elif key == "reset":
                reset_view()
                message_set("Back to plan view")
            elif key in ("plain", "dpta"):
                if grid:
                    plan(key)
            elif key == "both":
                compare_both()
            elif key == "execute":
                execute()
            elif key == "csv":
                export_csv()

def message_set(s):
    global message
    message = s

def key_down(event):
    global active_input, fresh
    if active_input is None:
        if event.key == pygame.K_SPACE:
            execute()
        return
    if event.key == pygame.K_RETURN:
        active_input = None
        generate_grid()
    elif event.key == pygame.K_BACKSPACE:
        inputs[active_input] = (
            ""
            if fresh
            else inputs[active_input][:-1]
        )
        fresh = False
    elif event.unicode.isdigit():
        if fresh:
            inputs[active_input] = ""
            fresh = False
        if len(inputs[active_input]) < max_len[active_input]:

            inputs[active_input] += event.unicode

def main():
    global last_tick_time
    generate_grid()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                click(event.pos)
            elif event.type == pygame.KEYDOWN:
                key_down(event)
        if playing and method:
            now = pygame.time.get_ticks()
            if now - last_tick_time >= TICK_DELAY:
                last_tick_time = now
                advance()
        draw()
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()
if __name__ == "__main__":
    main()