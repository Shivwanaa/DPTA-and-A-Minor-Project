import pygame
import subprocess
import sys
from grid import create_grid
from astar import astar, manhattan, euclidean, chebyshev

WIDTH = 1000
HEIGHT = 760
GRID_SIZE = 600
ROWS = 30
COLS = 30
CELL_SIZE = 20
PANEL_X = GRID_SIZE + 20

WHITE = (255, 255, 255)
BLACK = (30, 30, 30)
GRAY = (180, 180, 180)
GREEN = (0, 200, 0)
RED = (220, 0, 0)
DARK = (50, 50, 50)
BLUE = (80, 150, 255)
YELLOW = (255, 220, 70)
PURPLE = (170, 80, 220)

pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("A* Robot Path Finding")
clock = pygame.time.Clock()

font = pygame.font.SysFont(None, 25)
small_font = pygame.font.SysFont(None, 20)

grid, start, goal = create_grid(ROWS, COLS, 0.25)

heuristics = {
    "Manhattan": manhattan,
    "Euclidean": euclidean,
    "Chebyshev": chebyshev
}

selected_heuristic = "Manhattan"

path = None
explored = []
nodes_expanded = 0
execution_time = 0
path_length = 0

animation_running = False
animation_index = 0
path_animation_running = False
path_animation_index = 0

EXPLORATION_DELAY = 25
PATH_DELAY = 150
last_animation_time = 0

rows_input = "30"
cols_input = "30"
obstacle_input = "25"

input_active = False
active_input = None

manhattan_button = pygame.Rect(PANEL_X, 100, 250, 45)
euclidean_button = pygame.Rect(PANEL_X, 155, 250, 45)
chebyshev_button = pygame.Rect(PANEL_X, 210, 250, 45)

generate_button = pygame.Rect(PANEL_X, 290, 250, 45)
start_button = pygame.Rect(PANEL_X, 345, 250, 45)
reset_button = pygame.Rect(PANEL_X, 400, 250, 45)
export_button = pygame.Rect(PANEL_X, 455, 250, 45)

rows_box = pygame.Rect(PANEL_X + 145, 520, 105, 35)
cols_box = pygame.Rect(PANEL_X + 145, 565, 105, 35)
obstacle_box = pygame.Rect(PANEL_X + 145, 610, 105, 35)

def generate_grid():
    global grid, start, goal
    global ROWS, COLS, CELL_SIZE
    global path, explored
    global nodes_expanded, execution_time, path_length
    global animation_running, animation_index
    global path_animation_running, path_animation_index
    global last_animation_time

    try:
        ROWS = int(rows_input)
        COLS = int(cols_input)
    except ValueError:
        ROWS = 30
        COLS = 30

    ROWS = max(5, min(50, ROWS))
    COLS = max(5, min(50, COLS))

    CELL_SIZE = min(
        GRID_SIZE // COLS,
        GRID_SIZE // ROWS
    )

    try:
        percentage = float(obstacle_input)
    except ValueError:
        percentage = 25

    percentage = max(0, min(90, percentage))
    obstacle_probability = percentage / 100

    grid, start, goal = create_grid(
        ROWS,
        COLS,
        obstacle_probability
    )

    path = None
    explored = []
    nodes_expanded = 0
    execution_time = 0
    path_length = 0
    animation_running = False
    animation_index = 0
    path_animation_running = False
    path_animation_index = 0
    last_animation_time = 0

def start_search():
    global path, explored
    global nodes_expanded, execution_time, path_length
    global animation_running, animation_index
    global path_animation_running, path_animation_index
    global last_animation_time

    heuristic = heuristics[selected_heuristic]

    result = astar(
        grid,
        start,
        goal,
        heuristic
    )

    path = result["path"]
    explored = result["explored"]
    nodes_expanded = result["nodes_expanded"]
    execution_time = result["time"]
    path_length = result["path_length"]

    animation_running = True
    animation_index = 0
    path_animation_running = False
    path_animation_index = 0
    last_animation_time = pygame.time.get_ticks()

def export_csv():
    try:
        rows = int(rows_input)
        cols = int(cols_input)
        obstacle_probability = float(obstacle_input) / 100

        subprocess.Popen([
            sys.executable,
            "exp.py",
            str(rows),
            str(cols),
            str(obstacle_probability)
        ])

        print("CSV experiment started.")

    except ValueError:
        print("Invalid input values.")

def draw_grid():
    for r in range(ROWS):
        for c in range(COLS):
            x = c * CELL_SIZE
            y = r * CELL_SIZE

            if grid[r][c] == 1:
                cell_color = DARK
            else:
                cell_color = WHITE

            pygame.draw.rect(
                screen,
                cell_color,
                (x, y, CELL_SIZE, CELL_SIZE)
            )

    for r, c in explored[:animation_index]:
        x = c * CELL_SIZE
        y = r * CELL_SIZE

        pygame.draw.rect(
            screen,
            BLUE,
            (
                x + 2,
                y + 2,
                CELL_SIZE - 4,
                CELL_SIZE - 4
            )
        )

    if path is not None:
        for r, c in path[:path_animation_index]:
            x = c * CELL_SIZE
            y = r * CELL_SIZE

            pygame.draw.rect(
                screen,
                PURPLE,
                (
                    x + 2,
                    y + 2,
                    CELL_SIZE - 4,
                    CELL_SIZE - 4
                )
            )

    if (
        animation_running
        and animation_index > 0
        and animation_index <= len(explored)
    ):
        r, c = explored[animation_index - 1]
        x = c * CELL_SIZE
        y = r * CELL_SIZE

        pygame.draw.rect(
            screen,
            YELLOW,
            (
                x + 4,
                y + 4,
                CELL_SIZE - 8,
                CELL_SIZE - 8
            )
        )

    if (
        path_animation_running
        and path_animation_index > 0
        and path is not None
        and path_animation_index <= len(path)
    ):
        r, c = path[path_animation_index - 1]
        x = c * CELL_SIZE
        y = r * CELL_SIZE

        pygame.draw.rect(
            screen,
            YELLOW,
            (
                x + 4,
                y + 4,
                CELL_SIZE - 8,
                CELL_SIZE - 8
            )
        )

    r, c = start
    x = c * CELL_SIZE
    y = r * CELL_SIZE

    pygame.draw.rect(
        screen,
        GREEN,
        (
            x + 2,
            y + 2,
            CELL_SIZE - 4,
            CELL_SIZE - 4
        )
    )

    r, c = goal
    x = c * CELL_SIZE
    y = r * CELL_SIZE

    pygame.draw.rect(
        screen,
        RED,
        (
            x + 2,
            y + 2,
            CELL_SIZE - 4,
            CELL_SIZE - 4
        )
    )

    for r in range(ROWS):
        for c in range(COLS):
            x = c * CELL_SIZE
            y = r * CELL_SIZE

            pygame.draw.rect(
                screen,
                GRAY,
                (
                    x,
                    y,
                    CELL_SIZE,
                    CELL_SIZE
                ),
                1
            )

def draw_button(rect, text, selected=False):
    color = BLUE if selected else GRAY

    pygame.draw.rect(
        screen,
        color,
        rect,
        border_radius=5
    )

    text_surface = small_font.render(
        text,
        True,
        BLACK
    )

    text_rect = text_surface.get_rect(
        center=rect.center
    )

    screen.blit(
        text_surface,
        text_rect
    )

def draw_input_box(rect, text, active=False):
    border_color = BLUE if active else GRAY

    pygame.draw.rect(
        screen,
        WHITE,
        rect
    )

    pygame.draw.rect(
        screen,
        border_color,
        rect,
        2
    )

    input_surface = small_font.render(
        text,
        True,
        BLACK
    )

    screen.blit(
        input_surface,
        (
            rect.x + 8,
            rect.y + 7
        )
    )

def draw_panel():
    title = font.render(
        "A* Robot",
        True,
        WHITE
    )

    screen.blit(
        title,
        (PANEL_X, 30)
    )

    draw_button(
        manhattan_button,
        "Manhattan",
        selected_heuristic == "Manhattan"
    )

    draw_button(
        euclidean_button,
        "Euclidean",
        selected_heuristic == "Euclidean"
    )

    draw_button(
        chebyshev_button,
        "Chebyshev",
        selected_heuristic == "Chebyshev"
    )

    draw_button(
        generate_button,
        "Generate Grid"
    )

    draw_button(
        start_button,
        "Start Search"
    )

    draw_button(
        reset_button,
        "Reset"
    )

    draw_button(
        export_button,
        "Export CSV"
    )

    rows_text = small_font.render(
        "Rows:",
        True,
        WHITE
    )

    screen.blit(
        rows_text,
        (PANEL_X, 528)
    )

    draw_input_box(
        rows_box,
        rows_input,
        active_input == "rows"
    )

    cols_text = small_font.render(
        "Columns:",
        True,
        WHITE
    )

    screen.blit(
        cols_text,
        (PANEL_X, 573)
    )

    draw_input_box(
        cols_box,
        cols_input,
        active_input == "cols"
    )

    obstacle_text = small_font.render(
        "Obstacle %:",
        True,
        WHITE
    )

    screen.blit(
        obstacle_text,
        (PANEL_X, 618)
    )

    draw_input_box(
        obstacle_box,
        obstacle_input,
        active_input == "obstacle"
    )

    metrics = [
        f"Heuristic: {selected_heuristic}",
        f"Nodes: {nodes_expanded}",
        f"Path length: {path_length}",
        f"Time: {execution_time * 1000:.3f} ms"
    ]

    y = 670

    for text in metrics:
        surface = small_font.render(
            text,
            True,
            WHITE
        )

        screen.blit(
            surface,
            (PANEL_X, y)
        )

        y += 22

running = True

while running:
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            mouse_pos = event.pos

            if rows_box.collidepoint(mouse_pos):
                input_active = True
                active_input = "rows"

            elif cols_box.collidepoint(mouse_pos):
                input_active = True
                active_input = "cols"

            elif obstacle_box.collidepoint(mouse_pos):
                input_active = True
                active_input = "obstacle"

            else:
                input_active = False
                active_input = None

            if manhattan_button.collidepoint(mouse_pos):
                selected_heuristic = "Manhattan"

            elif euclidean_button.collidepoint(mouse_pos):
                selected_heuristic = "Euclidean"

            elif chebyshev_button.collidepoint(mouse_pos):
                selected_heuristic = "Chebyshev"

            elif generate_button.collidepoint(mouse_pos):
                generate_grid()

            elif start_button.collidepoint(mouse_pos):
                start_search()

            elif reset_button.collidepoint(mouse_pos):
                generate_grid()

            elif export_button.collidepoint(mouse_pos):
                export_csv()

        if (
            event.type == pygame.KEYDOWN
            and input_active
        ):
            if event.key == pygame.K_BACKSPACE:

                if active_input == "rows":
                    rows_input = rows_input[:-1]

                elif active_input == "cols":
                    cols_input = cols_input[:-1]

                elif active_input == "obstacle":
                    obstacle_input = obstacle_input[:-1]

            elif event.key == pygame.K_RETURN:
                generate_grid()
                input_active = False
                active_input = None

            elif event.unicode.isdigit():

                if (
                    active_input == "rows"
                    and len(rows_input) < 2
                ):
                    rows_input += event.unicode

                elif (
                    active_input == "cols"
                    and len(cols_input) < 2
                ):
                    cols_input += event.unicode

                elif (
                    active_input == "obstacle"
                    and len(obstacle_input) < 3
                ):
                    obstacle_input += event.unicode

    current_time = pygame.time.get_ticks()

    if animation_running:

        if (
            current_time - last_animation_time
            >= EXPLORATION_DELAY
        ):
            animation_index += 1
            last_animation_time = current_time

            if animation_index >= len(explored):
                animation_running = False

                if path is not None:
                    path_animation_running = True
                    path_animation_index = 0
                    last_animation_time = current_time

    elif path_animation_running:

        if (
            current_time - last_animation_time
            >= PATH_DELAY
        ):
            path_animation_index += 1
            last_animation_time = current_time

            if path_animation_index >= len(path):
                path_animation_running = False

    screen.fill(BLACK)
    draw_grid()
    draw_panel()

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
