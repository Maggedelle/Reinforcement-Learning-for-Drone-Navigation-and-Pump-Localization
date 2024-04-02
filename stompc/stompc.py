import random
import argparse
import os
import rclpy
import sys
import threading
import strategoutil as sutil
import time
import math
sys.path.insert(0, '../')

from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor, odom_publisher, map_processing
import time
from model_interface import QueueLengthController
from environment import generate_environment, build_uppaal_2d_array_string, unpack_environment
from utils import turn_drone
global offboard_control_instance
global odom_publisher_instance
global map_drone_tf_listener_instance
INITIAL_X = 0.0
INITIAL_Y = 0.0

half_PI_right = 1.57   # 90 degrees right
half_PI_left = -1.57   # 90 degrees left
full_PI_turn = 3.14    # 180 degress turn
e = 0.2
uppaa_e = 0.5
def activate_action(action):
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw
    match action:
        case 0:
            y-=1
        case 1:
            x+=1
        case 2:
            y+=1
        case 3:
            x-=1
        case 4:
            yaw = turn_drone(yaw, half_PI_left)
        case 5:
            yaw = turn_drone(yaw, half_PI_right)
        case 6:
            yaw = turn_drone(yaw,full_PI_turn)
        case _:
            print("unkown action")
            map,map_drone_index_x,map_drone_index_y, map_width, map_height, map_granularity = map_processing.process_map_data(x, y)
            return map,map_drone_index_x,map_drone_index_y, map_width, map_height, yaw, map_granularity


    offboard_control_instance.x = x
    offboard_control_instance.y = y
    offboard_control_instance.yaw = yaw
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())

    time.sleep(2)
    while((x-e > curr_x or curr_x > x+e) or (y-e > curr_y or curr_y > y+e)):
        time.sleep(0.5)
        curr_x = float(vehicle_odometry.get_drone_pos_x())
        curr_y = float(vehicle_odometry.get_drone_pos_y())

    map,map_drone_index_x,map_drone_index_y, map_width, map_height, map_granularity = map_processing.process_map_data(curr_x, curr_y)

    return map,map_drone_index_x,map_drone_index_y, map_width, map_height, yaw, map_granularity


def run(template_file, query_file, verifyta_path):
    print("running uppaal")
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "yaw", "map_swidth","height_map", "map", "granularity_map", "training_param"])
    # initial drone state
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = 0.0
    action = -1
    N = 0
    open_training = "1"
    optimize = "maxE"
    learning_param = "accum_reward"
    map,map_drone_index_x,map_drone_index_y, map_width, map_height, map_granularity = map_processing.process_map_data(x,y)
    controller.generate_query_file(optimize, learning_param,
                                   state_vars=["DroneController.DescisionState", unpack_environment(map, "map"), "x", "y"], 
                                   point_vars=["yaw"], 
                                   observables=["action", "x", "y", "yaw", "current_step_length"])


    L = 1000 # simulation length
    K = 1  # every K we will do MPC
    for k in range(L):
        K_START_TIME = time.time()
        # run plant

        #handle_action(next_action);
        
        #<- readings fra diverse sensor
        map,map_drone_index_x,map_drone_index_y, map_width, map_height, yaw, _ = activate_action(action)

        if k % K == 0:
            # at each MPC step we want a clean template copy
            # to insert variables
            controller.init_simfile()
            
            # insert current state into simulation template
            state = {
                "x": map_drone_index_x,
                "y": map_drone_index_y,
                "yaw":  yaw,
                "map": build_uppaal_2d_array_string("int", "map", map),
                "width_map": map_width,
                "height_map": map_height,
                "granularity_map": map_granularity,
                "training_param": open_training
            }
            #print(state)
            controller.insert_state(state)
            RUN_START_TIME = time.time()
            action, x,y, yaw,step_length = controller.run(
                queryfile=query_file,
                verifyta_path=verifyta_path)
            RUN_END_TIME = time.time()
            K_END_TIME = time.time()
            print(action,x,y,yaw,step_length)

            print("Iteration {} took: {}ms, training took: {} with array lengths of {}".format(k,(K_END_TIME-K_START_TIME)*10**3,(RUN_END_TIME-RUN_START_TIME)*10**3,N))
            
            #print("actions:",action_seq)


def init_image_bridge():
    print("Starting image bridge...")
    def run_bridge():
        print("image bridge started...")
        os.system('ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image')
    image_bridge_thread = threading.Thread(target=run_bridge)
    image_bridge_thread.start()

def init_clock_bridge():
    print("Starting clock bridge...")
    def run_bridge():
        print("clock bridge started...")
        os.system('ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock')
    clock_bridge_thread = threading.Thread(target=run_bridge)
    clock_bridge_thread.start()

def init_depth_camera_bridge():
    print("Starting depth_camera bridge...")
    def run_depth_camera():
        print("image depth_camera started...")
        os.system('ros2 run ros_gz_bridge parameter_bridge /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked --ros-args -r /depth_camera/points:=/cloud')
    depth_camera_brdige_thread = threading.Thread(target=run_depth_camera)
    depth_camera_brdige_thread.start()

def init_rclpy():
    print("initializing rclpy")
    rclpy.init(domain_id=2)

if __name__ == "__main__":
    
    init_rclpy()
    init_clock_bridge()
    offboard_control_instance = offboard_control.OffboardControl()
    offboard_control.init(offboard_control_instance)
    odom_publisher_instance = odom_publisher.FramePublisher()
    odom_publisher.init(odom_publisher_instance)
    map_drone_tf_listener_instance = vehicle_odometry.MapDroneFrameListener()
    vehicle_odometry.init_map_drone_tf(map_drone_tf_listener_instance)
    init_depth_camera_bridge()
    #init_image_bridge()

    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--template-file", default="drone_model_stompc_continuous.xml", 
        help="Path to Stratego .xml file model template")
    ap.add_argument("-q", "--query-file", default="query.q",
        help="Path to Stratego .q query file")
    ap.add_argument("-v", "--verifyta-path", default="/home/sw9-bois/uppaal-5.0.0-linux64/bin/verifyta", help=
        "Path to verifyta executable")
    args = ap.parse_args()

    base_path = os.path.dirname(os.path.realpath(__file__)) 
    template_file = os.path.join(base_path, args.template_file)
    query_file = os.path.join(base_path, args.query_file)
    while offboard_control_instance.has_aired == False:
        print(offboard_control_instance.vehicle_local_position.z)
        time.sleep(0.1)
    time.sleep(5)
    run(template_file, query_file, args.verifyta_path)
