from collections import deque
from classes import State, DroneSpecs, TrainingParameters

def get_action_from_bfs(state:State):

    rows, cols = len(state.map), len(state.map[0])
    queue = deque([(state.map_drone_index_y, state.map_drone_index_x, -1)])
    visited = [[False for _ in range(cols)] for _ in range(rows)]
    parents = [[None for _ in range(cols)] for _ in range(rows)]

    directions = [(10, 0), (0, 10), (-10, 0), (0, -10)]

    start_row, start_col = state.map_drone_index_y, state.map_drone_index_x
    visited[start_row][start_col] = True

    # BFS

    while queue:
        r, c, dir = queue.popleft()


        if state.map[r][c] == -1:
            path = []

            while (r, c) != (state.map_drone_index_y, state.map_drone_index_x):
                path.append(dir)
                r, c = parents[r][c]
            path.reverse()
            return path
        
        for i, (dr, dc) in enumerate(directions):
            nr, nc = r + dr, c + dc

            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                visited[nr][nc] = True
                parents[nr][nc] = (r, c)
                queue.append((nr,nc, i))

    
    return None 
