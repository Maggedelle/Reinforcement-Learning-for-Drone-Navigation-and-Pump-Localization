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
from dotenv import load_dotenv
load_dotenv()

from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor, odom_publisher, map_processing
import time
from model_interface import QueueLengthController
from environment import generate_environment, build_uppaal_2d_array_string, unpack_environment
from utils import turn_drone, shield_action, run_pump_detection
from classes import State, DroneSpecs, TrainingParameters
from maps import get_baseline_one_pump_config, get_baseline_two_pumps_config
global offboard_control_instance
global odom_publisher_instance
global map_drone_tf_listener_instance

ENV_DOMAIN = os.environ['DOMAIN']
ENV_VERIFYTA_PATH = os.environ['VERIFYTA_PATH']

INITIAL_X = 0.0
INITIAL_Y = 0.0

half_PI_right = 1.57   # 90 degrees right
half_PI_left = -1.57   # 90 degrees left
full_PI_turn = 3.14    # 180 degress turn
e = 0.1
uppaa_e = 0.5

drone_specs = DroneSpecs(drone_diameter=0.6,safety_range=0.4,laser_range=2,laser_range_diameter=2)
training_parameters = TrainingParameters(open=1, turning_cost=0.0, moving_cost=0.0, discovery_reward=10.0)
learning_args = {
    "max-iterations": "1",
    #"reset-no-better": "2",
    #"good-runs": "100",
    #"total-runs": "100",
    #"runs-pr-state": "100"
    }

global map_config
map_config = get_baseline_one_pump_config()

def get_current_state():
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw
    state = map_processing.process_map_data(x,y, map_config)
    state.yaw = yaw
    return state 

def activate_action(action):
    global map_config
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw
    match action:
        case 10:
            y-=0.5
        case 11:
            x+=0.5
        case 12:
            y+=0.5
        case 13:
            x-=0.5
        case 20:
            y-=1
        case 21:
            x+=1
        case 22:
            y+=1
        case 23:
            x-=1
        case 4:
            yaw = turn_drone(yaw, half_PI_left)
            time.sleep(2.5)
        case 5:
            yaw = turn_drone(yaw, half_PI_right)
            time.sleep(2.5)
        case 6:
            yaw = turn_drone(yaw,full_PI_turn)
            time.sleep(3.5)
        case _:
            print("unkown action")
            state = map_processing.process_map_data(x, y, map_config)
            state.yaw = yaw
            return state


    offboard_control_instance.x = x
    offboard_control_instance.y = y
    offboard_control_instance.yaw = yaw
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())


    while((x-e > curr_x or curr_x > x+e) or (y-e > curr_y or curr_y > y+e)):
        time.sleep(0.5)
        curr_x = float(vehicle_odometry.get_drone_pos_x())
        curr_y = float(vehicle_odometry.get_drone_pos_y())


    time.sleep(0.5)
    state = map_processing.process_map_data(curr_x, curr_y,  map_config)
    state.yaw = yaw
    map_config = run_pump_detection(state,map_config,drone_specs)
    return state


def run(template_file, query_file, verifyta_path):
    print("running uppaal")
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "yaw", "width_map","height_map", "map", "granularity_map", "open", "discovery_reward", "turning_cost", "moving_cost", "drone_diameter", "safety_range", "range_laser", "laser_range_diameter"])
    # initial drone state
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    action_seq = [-1]
    N = 0
    optimize = "maxE"
    learning_param = "accum_reward"
    state = map_processing.process_map_data(x,y, map_config)
    state.yaw = offboard_control_instance.yaw
    controller.generate_query_file(optimize, learning_param,
                                   state_vars=["DroneController.DescisionState"], 
                                   point_vars=["yaw", "x", "y"], 
                                   observables=["action"])


    total_time = 0.0
    k = 0  
    train = True
    horizon = 10
    while True:
        K_START_TIME = time.time()
        # run plant

        #handle_action(next_action);
        
        #<- Activate the current action and receive the updated state of the world


        if train == True or k % horizon == 0:
            # at each MPC step we want a clean template copy
            # to insert variables
            N = N + 1
            print("Beginning trainng for iteration {}".format(N))

            controller.init_simfile()
            
            # insert current state into simulation template
            uppaal_state = {
                "x": state.map_drone_index_x,
                "y": state.map_drone_index_y,
                "yaw":  state.yaw,
                "map": build_uppaal_2d_array_string("int", "map", state.map),
                "width_map": state.map_width,
                "height_map": state.map_height,
                "granularity_map": state.map_granularity,
                "open": training_parameters.open, 
                "discovery_reward": training_parameters.disovery_reward, 
                "turning_cost": training_parameters.turning_cost, 
                "moving_cost": training_parameters.moving_cost, 
                "drone_diameter": drone_specs.drone_diameter,
                "safety_range": drone_specs.safety_range,
                "range_laser": drone_specs.laser_range, 
                "laser_range_diameter": drone_specs.laser_range_diameter
            }
            #print(state)

            controller.insert_state(uppaal_state)
            train = False
            RUN_START_TIME = time.time()
            action_seq = controller.run(
                learning_args=learning_args,
                queryfile=query_file,
                verifyta_path=verifyta_path)
            k = 0
            RUN_END_TIME = time.time()
            K_END_TIME = time.time()
            iteration_time = (K_END_TIME-K_START_TIME)*10**3 / 1000
            learning_time = (RUN_END_TIME-RUN_START_TIME)*10**3 / 1000
            total_time += iteration_time / 60
            print("Training frIteration {} took: {:0.4f} seconds, training took: {:0.4f} seconds, total time spent: {:0.2f} minutes".format(N, iteration_time, learning_time, total_time))
            print("got action sequence from STRATEGO: ", action_seq)
        
        k=k+1
        if(len(action_seq) == 0):
            train = True
            k = 0
        else: 
            action = action_seq.pop(0)
            state = get_current_state()
            if(shield_action(action,state, drone_specs)):
                state = activate_action(action)
            else:
                print("shielded action: {}".format(action))
                
                time.sleep(3)
                state = get_current_state()
                if(shield_action(action,state,drone_specs)):
                    state = activate_action(action)
                else:
                    print("shielded action: {} twice, training again".format(action))
                    state = get_current_state()
                    train = True
                    k = 0
        
        
        
        
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
    rclpy.init(domain_id=int(ENV_DOMAIN))

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
    ap.add_argument("-v", "--verifyta-path", default=ENV_VERIFYTA_PATH, help=
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
