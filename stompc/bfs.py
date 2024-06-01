from collections import deque
from classes import State, DroneSpecs, TrainingParameters
from utils import check_if_drone_can_see_pump, shield_action
def get_path_from_bfs(state:State, drone_specs: DroneSpecs):

    rows, cols = len(state.map), len(state.map[0])
    queue = deque([(state.map_drone_index_y, state.map_drone_index_x, -1)])
    visited = [[False for _ in range(cols)] for _ in range(rows)]
    parents = [[None for _ in range(cols)] for _ in range(rows)]

    directions = [(10, 0), (0, 10), (-10, 0), (0, -10)]

    start_row, start_col = state.map_drone_index_y, state.map_drone_index_x
    visited[start_row][start_col] = True

    path_to_poi = None
    path_to_unkown_cell = None

    # BFS

    while queue:
        r, c, dir = queue.popleft()



        #found a poi
        if state.map[r][c] == 2:
            path_to_poi = []
            while (r,c) != (state.map_drone_index_y, state.map_drone_index_x):
                path_to_poi.append(dir)
                r, c = parents[r][c]
            path_to_poi.reverse()
            return path_to_poi
        
        #found a unkown cell
        if state.map[r][c] == -1 and path_to_unkown_cell:
            path_to_unkown_cell = []

            while (r, c) != (state.map_drone_index_y, state.map_drone_index_x):
                path_to_unkown_cell.append(dir)
                r, c = parents[r][c]
            path_to_unkown_cell.reverse()
            
        


        for i, (dr, dc) in enumerate(directions):
            nr, nc = r + dr, c + dc

            #add the cell to queue if it has not been visited, and it is safe to take
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc] and not shield_action(i + 10, state, drone_specs):
                visited[nr][nc] = True
                parents[nr][nc] = (r, c)
                queue.append((nr,nc, i))

    
    if path_to_unkown_cell != None:
        return path_to_unkown_cell
    else:
        print("Didn't find any POI or unkown cells")
        return None 
