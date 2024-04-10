import math
from functools import total_ordering

class State: 
    yaw = 0.0
    def __init__(self, map:list, map_drone_index_x:int,map_drone_index_y:int, map_width:int, map_height:int, map_granularity:float):
        self.map = map
        self.map_drone_index_x = map_drone_index_x
        self.map_drone_index_y = map_drone_index_y
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
    def __init__(self, open:int, turning_cost: float, moving_cost: float, discovery_reward: float):
        self.open = open
        self.turning_cost = turning_cost
        self.moving_cost = moving_cost
        self.disovery_reward = discovery_reward

@total_ordering
class Point:
    """
    Simple 2-dimensional point. In the code x and y are switched, so that it matches coordinates in for our matrix.
    Orientation in the matrix:
                W: +y
        N: -x ----|---- E: +x
                S: -y
    """
    def __init__(self, x, y):
        self.x = y
        self.y = x

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, n):
        return Point(self.x * n, self.y * n)

    def __div__(self, n):
        return Point(self.x / n, self.y / n)

    def __neg__(self):
        return Point(-self.x, -self.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        return not self == other

    def __lt__(self, other):
        return self.length < other.length

    def __str__(self):
        return "({}, {})".format(self.x, self.y)

    def __repr__(self):
        return "Point({}, {})".format(self.x, self.y)

    def __hash__(self):
        return hash(tuple((self.x, self.y)))

    def dist(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def dist_manhattan(self, other):
        return abs(self.x - other.x) + abs(self.y - other.y)
    
    def dist_chess(self, other):
        return max(abs(self.x - other.x), abs(self.y - other.y))
    
    def dist_chebyshev(self, other):
        return self.dist_chess(other)

    def angle(self, to=None):
        if to is None:
            return math.atan2(self.y, self.x)
        return math.atan2(self.y - to.y, self.x - to.x)

    def rotate(self, turns):
        """Returns the rotation of the Point around (0, 0) `turn` times clockwise."""
        turns = turns % 4

        if turns == 1:
            return Point(self.y, -self.x)
        elif turns == 2:
            return Point(-self.x, -self.y)
        elif turns == 3:
            return Point(-self.y, self.x)
        else:
            return self

    @property
    def manhattan(self):
        return abs(self.x) + abs(self.y)
    
    @property
    def chess(self):
        return max(abs(self.x), abs(self.y))
    
    @property
    def chebyshev(self):
        return self.chess

    @property
    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def neighbours_4(self):
        return [self + p for p in DIRS_4]

    def neighbors_4(self):
        return self.neighbours_4()

    def neighbours(self):
        return self.neighbours_4()

    def neighbors(self):
        return self.neighbours()

    def neighbours_8(self):
        return [self + p for p in DIRS_8]

    def neighbors_8(self):
        return self.neighbours_8()
    
DIRS_8 = [
    Point(-1, 0),    # N
    Point(-1, -1),    # NE
    Point(0, -1),    # E
    Point(1, -1),   # SE
    Point(1, 0),   # S
    Point(1, 1),  # SW
    Point(0, 1),   # W
    Point(-1, 1),   # NW
]

DIRS_4 = DIRS = [
    Point(-1, 0),   # north
    Point(0, -1),   # east
    Point(1, 0),  # south
    Point(0, 1),  # west
]