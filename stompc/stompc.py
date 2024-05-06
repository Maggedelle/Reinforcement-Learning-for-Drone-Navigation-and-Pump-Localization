import random
import argparse
import os
import rclpy
import sys
import threading
import strategoutil as sutil
import time
import math
from multiprocessing import Process, Queue, Pipe
sys.path.insert(0, '../')
from dotenv import load_dotenv
load_dotenv()

from gz_utils import run_gz, kill_gz, run_xrce_agent, kill_xrce_agent, run_launch_file, kill_launch_file
from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor, odom_publisher, map_processing
import time
from model_interface import QueueLengthController
from bridges import init_rclpy, shutdown_rclpy
from environment import generate_environment
from utils import turn_drone, shield_action, unpack_array, build_uppaal_2d_array_string, run_pump_detection, check_map_closed 
from classes import State, DroneSpecs, TrainingParameters
from maps import get_baseline_one_pump_config, get_baseline_two_pumps_config
global offboard_control_instance
global odom_publisher_instance
global map_drone_tf_listener_instance

ENV_DOMAIN = os.environ['DOMAIN']
ENV_VERIFYTA_PATH = os.environ['VERIFYTA_PATH']
ENV_GZ_PATH = os.environ['GZ_PATH']
ENV_LAUNCH_FILE_PATH = os.environ['LAUNCH_FILE_PATH']

INITIAL_X = 0.0
INITIAL_Y = 0.0

half_PI_right = 1.57   # 90 degrees right
half_PI_left = -1.57   # 90 degrees left
full_PI_turn = 3.14    # 180 degress turn
e_turn = 0.05
e_move = 0.1
uppaa_e = 0.5

drone_specs = DroneSpecs(drone_diameter=0.6,safety_range=0.4,laser_range=2,laser_range_diameter=2)
training_parameters = TrainingParameters(open=1, turning_cost=20.0, moving_cost=20.0, discovery_reward=10.0, pump_exploration_reward=1000.0)
learning_args = {
    "max-iterations": "6",
    "reset-no-better": "2",
    "good-runs": "100",
    "total-runs": "100",
    "runs-pr-state": "100"
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
 
def run_action_seq(actions:list):
    """
    Returns TRUE if all actions was successfully executed
    Returns FALSE if all actions was not successfully executed.
    """
    while(len(actions) > 0):
        action_was_activated = activate_action_with_shield(actions.pop(0))
        if(action_was_activated == False):
            return False
    return True


def activate_action_with_shield(action):
    """
    Returns TRUE if action is activated
    Returns FALSE if action is not activated / is not safe.
    """
    state = get_current_state()
    if(shield_action(action,state, drone_specs)):
        state = activate_action(action)
    else:
        print("shielded action: {}".format(action))
        run_action_seq([4,4,4,4])
        state = get_current_state()
        if(shield_action(action,state,drone_specs)):
            activate_action(action)
        else:
            print("shielded action: {} twice, training again".format(action))
            return False
    
    return True

def predict_state_based_on_action_seq(action_seq):
    """
    Returns the predicted state of the drone, by taking the action_seq
    Returns None if the action_seq contains an unkown action.
    """
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw

    action_seq_copy = [act for act in action_seq]

    while(len(action_seq_copy) > 0):
        action = action_seq_copy.pop(0)
        try:
            x,y,yaw = get_drone_pos_based_on_action(action,x,y,yaw)
        except:
            print("could not predict state due to unkown action: " + action)
            return None
    state = map_processing.process_map_data(x, y,  map_config)
    state.yaw = yaw
    return state
        

    

def get_drone_pos_based_on_action(action,x,y,yaw):
    """
    Returns x,y,yaw based on some action.
    Raises exception if action is unkown.
    """
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
        case 5:
            yaw = turn_drone(yaw, half_PI_right)
        case 6:
            yaw = turn_drone(yaw,full_PI_turn)
        case _:
            raise Exception("Unkown action")
    return x,y,yaw

def activate_action(action):
    global map_config
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw
    action_is_move = False
    try:
        x,y,yaw = get_drone_pos_based_on_action(action,x,y,yaw)
    except:
        print("unkown action")
        state = map_processing.process_map_data(x, y, map_config)
        state.yaw = yaw
        return state


    action_is_move = action > 6
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())

    if action_is_move:
        offboard_control_instance.x = x
        offboard_control_instance.y = y
        while((x-e_move> curr_x or curr_x > x+e_move) or (y- e_move > curr_y or curr_y > y+e_move)):
            time.sleep(0.1)
            curr_x = float(vehicle_odometry.get_drone_pos_x())
            curr_y = float(vehicle_odometry.get_drone_pos_y())
    else:
        offboard_control_instance.yaw = yaw
        while((yaw - e_turn > odom_publisher_instance.yaw or odom_publisher_instance.yaw > yaw + e_turn)):
            time.sleep(0.1)
            

    
    state = map_processing.process_map_data(curr_x, curr_y,  map_config)
    state.yaw = yaw
    map_config = run_pump_detection(state,map_config,drone_specs)
    return state


def run(template_file, query_file, verifyta_path):
    print("running uppaal")
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "yaw", "width_map","height_map", "map", "granularity_map", "open", "discovery_reward", "turning_cost", "moving_cost", "drone_diameter", "safety_range", "range_laser", "laser_range_diameter", "pump_exploration_reward"])
    # initial drone state
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    action_seq = []
    N = 0
    optimize = "maxE"
    learning_param = "accum_reward - time"
    state = map_processing.process_map_data(x,y, map_config)
    state.yaw = offboard_control_instance.yaw
    controller.generate_query_file(optimize, learning_param,
                                   state_vars=["DroneController.DescisionState"], 
                                   point_vars=["yaw", "x", "y"], 
                                   observables=["action"])


    total_time = 0.0
    k = 0
    actions_left_to_trigger_learning = 3  
    train = True
    horizon = 10
    while not all(pump.has_been_discovered for pump in map_config.pumps + map_config.fake_pumps) or not check_map_closed(state, 0.5):
        K_START_TIME = time.time()
    
        if train == True or k % horizon == 0:

            N = N + 1
            print("Beginning trainng for iteration {}".format(N))

            controller.init_simfile()
            
            if(len(action_seq) == actions_left_to_trigger_learning):
                state = predict_state_based_on_action_seq(action_seq)
            

            # insert current state into simulation template
            uppaal_state = {
                "x": state.map_drone_index_x,
                "y": state.map_drone_index_y,
                "yaw":  state.yaw,
                "map": build_uppaal_2d_array_string("int", "map",  state.map),
                "width_map": state.map_width,
                "height_map": state.map_height,
                "granularity_map": state.map_granularity,
                "open": training_parameters.open, 
                "discovery_reward": training_parameters.disovery_reward, 
                "turning_cost": training_parameters.turning_cost, 
                "moving_cost": training_parameters.moving_cost,
                "pump_exploration_reward": training_parameters.pump_exploration_reward, 
                "drone_diameter": drone_specs.drone_diameter,
                "safety_range": drone_specs.safety_range,
                "range_laser": drone_specs.laser_range, 
                "laser_range_diameter": drone_specs.laser_range_diameter
            }
            controller.insert_state(uppaal_state)
            train = False
            RUN_START_TIME = time.time()
            
            
            parent_conn, child_conn = Pipe()
            t = Process(target=controller.run, args=(child_conn,query_file,learning_args,verifyta_path,))
            t.start()
            while t.is_alive():
                if(len(action_seq) > 0):
                    all_actions_were_activated = run_action_seq(action_seq)
                    action_seq = []
                    if(all_actions_were_activated == False):
                        train = True
                        t.terminate()
                        t.join()
                else:
                    run_action_seq([4,4,4,4])
                t.join(0.2)
            action_seq = list(parent_conn.recv())
            if(train == True):
                state = get_current_state()
                continue
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
            action_was_activated = activate_action_with_shield(action)
            state = get_current_state()

            if action_was_activated == False:
                train = True
                k = 0
                action_seq = []
            elif len(action_seq) == actions_left_to_trigger_learning:
                train = True
                k = 0

    print("Drone finsihed. Turning off drone")
    offboard_control_instance.shutdown_drone = True

if __name__ == "__main__":
    init_rclpy()
    run_xrce_agent()
    run_gz(GZ_PATH=ENV_GZ_PATH)
    time.sleep(15)
    run_launch_file(LAUNCH_PATH=ENV_LAUNCH_FILE_PATH)
    offboard_control_instance = offboard_control.OffboardControl()
    offboard_control.init(offboard_control_instance)
    odom_publisher_instance = odom_publisher.FramePublisher()
    odom_publisher.init(odom_publisher_instance)
    map_drone_tf_listener_instance = vehicle_odometry.MapDroneFrameListener()
    vehicle_odometry.init_map_drone_tf(map_drone_tf_listener_instance)

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
    run(template_file, query_file, args.verifyta_path)
    kill_gz()
    kill_xrce_agent()
    kill_launch_file()
    shutdown_rclpy()