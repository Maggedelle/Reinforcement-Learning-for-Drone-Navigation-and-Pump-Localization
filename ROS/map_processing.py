import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.srv import GetMap
import numpy as np
import threading
import math
class MapServiceCaller(Node):
    def __init__(self) -> None:
        super().__init__('map_listener')
        self.cli = self.create_client(GetMap, '/slam_toolbox/dynamic_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()


    def send_request(self):
        self.future = self.cli.call_async(self.req)
        



def process_map_data(drone_x, drone_y):

    msg = None
    map_service_instance = MapServiceCaller()
    map_service_instance.send_request()
    while rclpy.ok():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(map_service_instance)
        executor.spin_once()
        if map_service_instance.future.done():
            try:
                response = map_service_instance.future.result()
            except Exception as e:
                map_service_instance.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
               msg = response.map
            break

    print("map service called")
    width = msg.info.width
    matrix = []
    row = []
    data = msg.data
    x_offset = abs(math.floor((msg.info.origin.position.x / msg.info.resolution)))
    y_offset = abs(math.floor((msg.info.origin.position.y / msg.info.resolution)))
    print(msg.info.origin.position.x, msg.info.origin.position.y)
    print(x_offset,y_offset)
    x_index = math.floor((drone_x) / msg.info.resolution) + x_offset
    y_index = math.floor(((drone_y * -1)) / msg.info.resolution) + y_offset

    print(x_index, y_index)
    for i in range(0, len(data), width):
        row = data[i:i + width]
        matrix.append(row)


    x = 0
    y = 0
    print("drone x,y in map: ({},{})".format(x_index,y_index))
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
            testfile.write(', '.join(string_row) + '\n')
            y = 0
            
            
    return msg.info.origin, msg.info.resolution




