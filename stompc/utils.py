from classes import State, DroneSpecs, Point, DIRS_8, DIRS_4
import math
PI_upper = 3.14
PI_lower = -3.14
PI_half_pos = 1.57
PI_half_neg = -1.57

def turn_drone(yaw, yaw_dx):
    if yaw >= PI_upper and yaw_dx > 0: 
        yaw = PI_lower + yaw_dx
    elif yaw <= PI_lower and yaw_dx < 0:
        yaw = PI_upper + yaw_dx
    elif yaw + yaw_dx > PI_upper: 
        if yaw == PI_half_pos or yaw == PI_half_neg:
            yaw = yaw * -1
        else:
            yaw = PI_lower + yaw_dx
    else:
        yaw = yaw + yaw_dx
        
    return yaw


def shield_action(action: int, state:State, drone_specs: DroneSpecs) -> bool:
    """
    Returns TRUE if action is SAFE.
    Returns FALSE if action is UNSAFE.
    """
    step_length = 0
    if(action > 14):
        step_length = 1.0
    else:
        step_length = 0.5
    N_cells_in_dir = int(step_length / state.map_granularity)
    drone_cells_to_cover = int((drone_specs.drone_diameter) / state.map_granularity)
    safety_range_cells = int(drone_specs.safety_range / state.map_granularity)
    
    if drone_cells_to_cover % 2 == 0:
        drone_cells_to_cover += 1
    
    
    match action:
        case 10 | 20:
            lower_bound_x = int(state.map_drone_index_x - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_x = int(state.map_drone_index_x +  (drone_cells_to_cover / 2) + safety_range_cells)
            upper_bound_y = int(state.map_drone_index_y + N_cells_in_dir + safety_range_cells)


            if lower_bound_x < 0 or upper_bound_x > state.map_width or upper_bound_y > state.map_height:
                return False
            
            for i in range(lower_bound_x, upper_bound_x):
                for j in range (state.map_drone_index_y + 1, upper_bound_y):
                    if state.map[j][i] == 100 or state.map[j][i] == -1:
                        return False
                    
                
            
            return True
        case 11 | 21:
            lower_bound_y = int(state.map_drone_index_y - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_y = int(state.map_drone_index_y +  (drone_cells_to_cover / 2) + safety_range_cells)
            upper_bound_x = int(state.map_drone_index_x + N_cells_in_dir + safety_range_cells)


            if lower_bound_y < 0 or upper_bound_y > state.map_height or upper_bound_x > state.map_width:
                return False
            
            for i in range(state.map_drone_index_x + 1, upper_bound_x):
                for j in range (lower_bound_y, upper_bound_y):
                    if state.map[j][i] == 100 or state.map[j][i] == -1:
                        return False
                    
            return True
        case 12 | 22:
            lower_bound_x = int(state.map_drone_index_x - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_x = int(state.map_drone_index_x +  (drone_cells_to_cover / 2) + safety_range_cells)
            lower_bound_y = int(state.map_drone_index_y - N_cells_in_dir - safety_range_cells)

          

            if lower_bound_x < 0 or upper_bound_x > state.map_width or lower_bound_y < 0:
                return False
            

            for i in range(lower_bound_x, upper_bound_x):
                for j in reversed(range(lower_bound_y, state.map_drone_index_y - 1)):
                    if state.map[j][i] == 100 or  state.map[j][i] == -1: 
                        return False
                    
            return True
        case 13 | 23:
            lower_bound_y = int(state.map_drone_index_y - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_y = int(state.map_drone_index_y +  (drone_cells_to_cover / 2) + safety_range_cells)
            lower_bound_x = int(state.map_drone_index_x - N_cells_in_dir - safety_range_cells)

          

            if lower_bound_y < 0 or upper_bound_y > state.map_height or lower_bound_x < 0:
                return False
            

            for i in reversed(range(lower_bound_x, state.map_drone_index_x - 1)):
                for j in range(lower_bound_y, upper_bound_y):
                    if state.map[j][i] == 100 or  state.map[j][i] == -1: 
                        return False
                    
            return True
        case _:
            return True

def build_uppaal_2d_array_string(type, name, array):
    """
    Builds a 2D array to be inserted into uppaal from a python lists of lists.

    @type: denotes the type of the array when it's inserted into uppaal
    @name: the name the array is going to have in uppaal
    @array: the content of the array
    """
    uppaal_array = "{} {}[{}][{}] = ".format(type, name, len(array), len(array[0]))
    uppaal_array += "{\n"

    lst_strings = []
    for lst_ele in array:
        arr_string = "  {"
        arr_string += ','.join([str(x) for x in lst_ele])
        arr_string += "}"
        lst_strings.append(arr_string)
    uppaal_array += ',\n'.join(lst_strings) + "\n}"


    return uppaal_array

def unpack_array(array, array_name):
    """
    Unpacks an array so that it can be used in training query
    
    @arr: the array to unpack
    @arr_name: the name of the array in uppaal
    """
    lst_string = []
    for i in range(0, len(array)):
        for j in range(0, len(array[0])):
            lst_string.append("{}[{}][{}]".format(array_name, i,j))

    return ",".join(lst_string) 

N = Point(-1, 0)
NE = Point(-1, -1)
E = Point(0, -1)
SE = Point(1, -1)
S = Point(1, 0)
SW = Point(1, 1)
W = Point(0, 1)
NW = Point(-1, 1)

def measure_closure(map: list, x: int, y: int) -> float:
    """
    measure_closure(map: list, x: int, y: int) -> float

    Returns the measure of how closed the map is.
    The measure is saying how long the gaps in the walls are compared to the length of the wall.
    """

    # Find the nearest wall. This is used as the starting point for the measure
    drone_pos = Point(x,y)
    wall_start = None
    cells_from_drone = 0
    for dir in DIRS_4:
        temp_point = drone_pos
        while True:
            #TODO: Her skal jeg lave så jeg finder hvilken dir der er tættest på en væg
            NotImplemented