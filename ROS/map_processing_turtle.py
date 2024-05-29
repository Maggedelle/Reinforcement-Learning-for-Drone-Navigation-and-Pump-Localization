import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.srv import GetMap
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
import threading
import math
import os
import sys

sys.path.insert(0, '../')
from stompc.classes import State, MapConfig
from stompc.utils import get_map_index_of_pump

class MapProcessing(Node):
    def __init__(self) -> None:
        super().__init__('map_proccessing_turtle')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map = None
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile)
         
    def map_callback(self, msg):
        self.map = msg

    def process_map_data(self, drone_x: float, drone_y: float, map_config: MapConfig) -> State:
        if(self.map == None): return None
        msg = self.map
        #print("map service called")
        width = msg.info.width
        height = msg.info.height
        granularity = round(msg.info.resolution,2)
        matrix = []
        row = []
        data = msg.data

        all_pumps = map_config.fake_pumps + map_config.pumps

        x_offset = abs(math.floor((msg.info.origin.position.x / granularity)))
        y_offset = abs(math.floor((msg.info.origin.position.y / granularity)))

        x_index = math.floor((drone_x) / granularity) + x_offset
        y_index = math.floor(((drone_y)) / granularity) + y_offset

        for i in range(0, len(data), width):
            row = data[i:i + width]
            transformed_row = []
            for num in row:
                if num == -1:
                    transformed_row.append(-1)
                elif num > 65:
                    transformed_row.append(100)
                else:
                    transformed_row.append(0)
            matrix.append(transformed_row)

        state = State(matrix, x_index, y_index, width, height, granularity, x_offset, y_offset)
        for pump in all_pumps:
            pump_x_index, pump_y_index = get_map_index_of_pump(state,pump)
            print(pump_x_index, pump_y_index)
            if (pump_x_index >= state.map_width or pump_y_index >= state.map_height 
                or pump_x_index < 0 or pump_y_index < 0):
                continue
            elif pump.has_been_discovered == False:
                matrix[pump_y_index][pump_x_index] = 2
            elif pump.has_been_discovered == True and pump in map_config.fake_pumps:
                matrix[pump_y_index][pump_x_index] = 0
            elif pump.has_been_discovered == True and pump in map_config.pumps:
                matrix[pump_y_index][pump_x_index] = 3


        x = 0
        y = 0
        with open('matrix.txt', 'w') as testfile:
            for row in matrix:
                x=x+1
                string_row = []
                for a in row:
                    y=y+1
                    if x == y_offset and y == x_offset:
                        string_row.append("M")
                    if x == y_index and y == x_index:
                        string_row.append("*")
                    elif a == -1:
                        string_row.append("?")
                    elif a == 0:
                        string_row.append("+")
                    elif a == 100:
                        string_row.append("-")
                    elif a == 2:
                        string_row.append("!")
                    else: print(a)
                testfile.write(', '.join(string_row) + '\n')
                y = 0
                
        return state

            
        
def init(map_processing_instance, args=None) -> None:
    print('Starting map node...')
   

    def run_odom_publisher():
       executor = rclpy.executors.SingleThreadedExecutor()
       executor.add_node(map_processing_instance)
       executor.spin()
    offboard_thread = threading.Thread(target=run_odom_publisher)
    offboard_thread.start()






