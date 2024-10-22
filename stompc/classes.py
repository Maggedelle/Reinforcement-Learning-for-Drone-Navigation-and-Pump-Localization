class State: 
    yaw = 0.0
    def __init__(self, map:list, map_drone_index_x:int,map_drone_index_y:int, map_width:int, map_height:int, map_granularity:float, map_odom_index_x:int, map_odom_index_y: int):
        self.map = map
        self.map_drone_index_x = map_drone_index_x
        self.map_drone_index_y = map_drone_index_y
        self.map_odom_index_x = map_odom_index_x
        self.map_odom_index_y = map_odom_index_y
        self.map_width = map_width
        self.map_height = map_height
        self.map_granularity = map_granularity



class DroneSpecs:
    def __init__(self, drone_diameter:float, safety_range:float, laser_range:int, laser_range_diameter: int):
        self.drone_diameter = drone_diameter
        self.safety_range = safety_range
        self.laser_range = laser_range
        self.laser_range_diameter = laser_range_diameter


class TrainingParameters:
    def __init__(self, open:int, turning_cost: float, moving_cost: float, discovery_reward: float, pump_exploration_reward: float):
        self.open = open
        self.turning_cost = turning_cost
        self.moving_cost = moving_cost
        self.disovery_reward = discovery_reward
        self.pump_exploration_reward = pump_exploration_reward



class Pump:
    has_been_discovered = False
    def __init__(self, x:float, y:float) -> None:
        self.x = x
        self.y = y

        


class MapConfig:
    def __init__(self, pumps:list[Pump], fake_pumps:list[Pump], n_cells_in_area: int):
        self.pumps = pumps
        self.fake_pumps = fake_pumps
        self.n_cells_in_area = n_cells_in_area



