import random
def create_grid(rows, cols, obstacle_probability=0.25):
    grid = []
    for r in range(rows):
        row = []
        for c in range(cols):
            if random.random() < obstacle_probability:
                row.append(1)
            else:
                row.append(0)
        grid.append(row)
    while True:
        start = (
            random.randint(0, rows - 1),
            random.randint(0, cols - 1)
        )
        if grid[start[0]][start[1]] == 0:
            break
    while True:
        goal = (
            random.randint(0, rows - 1),
            random.randint(0, cols - 1)
        )
        if grid[goal[0]][goal[1]] == 0 and goal != start:
            break
    return grid, start, goal