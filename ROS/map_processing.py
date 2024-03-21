import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.srv import GetMap
import numpy as np
import threading

class MapServiceCaller(Node):
    def __init__(self) -> None:
        super().__init__('map_listener')
        self.cli = self.create_client(GetMap, '/slam_toolbox/dynamic_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()
    

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        



def process_map_data():

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

    print(len(data))
    print(width * msg.info.height)
    for i in range(0, len(data), width):
        row = data[i:i + width]
        matrix.append(row)

    with open('matrix.txt', 'w') as testfile:
        for row in matrix:
            string_row = []
            for a in row:
                if a == -1:
                    string_row.append("?")
                elif a == 0:
                    string_row.append("+")
                elif a == 100:
                    string_row.append("-")
            testfile.write(', '.join(string_row) + '\n')




