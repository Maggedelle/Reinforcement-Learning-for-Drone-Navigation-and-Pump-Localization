import random
import argparse
import os
import rclpy
import sys
import threading
import strategoutil as sutil
sys.path.insert(0, '../')
from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor, odom_publisher, map_processing
from ROS import offboard_control_turtle
import time
import math
from multiprocessing import Process, Queue, Pipe
from dotenv import load_dotenv
load_dotenv()

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
global map_config

global south_yaw
global north_yaw 
global west_yaw 
global east_yaw
map_config = get_baseline_one_pump_config()
ENV_VERIFYTA_PATH = os.environ['VERIFYTA_PATH']

half_PI_right = 1.57   # 90 degrees right
half_PI_left = -1.57   # 90 degrees left
full_PI_turn = 3.14    # 180 degress turn
e_turn = 0.05
e_move = 0.1
uppaa_e = 0.5

drone_specs = DroneSpecs(drone_diameter=0.07,safety_range=0.1,laser_range=2,laser_range_diameter=2)
training_parameters = TrainingParameters(open=1, turning_cost=20.0, moving_cost=20.0, discovery_reward=10.0, pump_exploration_reward=1000.0)
learning_args = {
    "max-iterations": "6",
    "reset-no-better": "2",
    "good-runs": "100",
    "total-runs": "100",
    "runs-pr-state": "100"
    }


def predict_state_based_on_action_seq(action_seq):
    """
    Returns the predicted state of the drone, by taking the action_seq
    Returns None if the action_seq contains an unkown action.
    """
    x = offboard_control_instance.x
    y = offboard_control_instance.y
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
        
def get_current_state():
    x = offboard_control_instance.x
    y = offboard_control_instance.y
    yaw = offboard_control_instance.yaw
    state = map_processing.process_map_data(x,y, map_config)
    state.yaw = yaw
    return state
    

def is_yaw_in_range(yaw_to_check):
    yaw = offboard_control_instance.yaw
    print(yaw_to_check, yaw)
    if yaw_to_check - 0.1 > yaw or yaw > yaw_to_check + 0.1: 
        return False
    else:
        return True
    

def calculate_yaw(dir):
    print(south_yaw,west_yaw,north_yaw,east_yaw)
    if is_yaw_in_range(south_yaw):
        print("im in south")
        if dir == "RIGHT":
            return west_yaw
        else: return east_yaw
    elif is_yaw_in_range(north_yaw):
        print("im in north")
        if dir == "RIGHT":
            return east_yaw
        else: return west_yaw
    elif is_yaw_in_range(west_yaw):
        print("im in west")
        if dir == "RIGHT":
            return north_yaw
        else: return south_yaw
    elif is_yaw_in_range(east_yaw):
        print("im in east")
        if dir == "RIGHT":
            return south_yaw
        else: return north_yaw



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
            yaw = calculate_yaw("LEFT")
        case 5:
            yaw = calculate_yaw("RIGHT")
        case _:
            raise Exception("Unkown action")
    return x,y,yaw


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


def get_move_distance_from_action(action):
    if(action < 6):
        return 0.0
    elif(action > 6 and action < 15):
        return 0.5
    else: return 1.0


def get_all_yaw():
    global south_yaw
    global north_yaw
    global east_yaw
    global west_yaw
    south_yaw = offboard_control_instance.yaw
    west_yaw = south_yaw + half_PI_right
    west_yaw = (west_yaw + math.pi) % (2 * math.pi) - math.pi   
    north_yaw = west_yaw + half_PI_right
    north_yaw = (north_yaw + math.pi) % (2 * math.pi) - math.pi
    east_yaw = north_yaw + half_PI_right
    east_yaw = (east_yaw + math.pi) % (2 * math.pi) - math.pi

def activate_action(action):
    global map_config
 
    
    action_is_move = action > 6
    
    if action_is_move:
        distance = get_move_distance_from_action(action)
        offboard_control_instance.move_robot(distance, None, 0.2)
        while(offboard_control_instance.is_running_action == True):
            time.sleep(0.1)
    else:
        if action == 4:
            radians = calculate_yaw("LEFT")
            vel = -0.4
        else: 
            radians = calculate_yaw("RIGHT")
            vel = 0.4
        offboard_control_instance.move_robot(None, radians, vel)
        while(offboard_control_instance.is_running_action == True):
            time.sleep(0.1)
        print("ran one action")



    

def run(template_file, query_file, verifyta_path):
    controller = QueueLengthController(
    templatefile=template_file,
    state_names=["x", "y", "yaw", "width_map","height_map", "map", "granularity_map", "open", "discovery_reward", "turning_cost", "moving_cost", "drone_diameter", "safety_range", "range_laser", "laser_range_diameter", "pump_exploration_reward", "north_yaw", "south_yaw", "west_yaw", "east_yaw"])
    # initial drone state
    x = offboard_control_instance.x
    y = offboard_control_instance.y

    disable_stompc = False

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

    
    get_all_yaw()
    #run_action_seq([4,4,4,4])
    
    if(disable_stompc == False):
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
                    "laser_range_diameter": drone_specs.laser_range_diameter,
                    "north_yaw": north_yaw,
                    "south_yaw": south_yaw,
                    "west_yaw": west_yaw,
                    "east_yaw": east_yaw
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

if __name__ == "__main__":
    init_rclpy()

    offboard_control_instance = offboard_control_turtle.OffboardControlTurtle()
    offboard_control_turtle.init(offboard_control_instance)

    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--template-file", default="drone_model_stompc_continuous_turtle.xml", 
        help="Path to Stratego .xml file model template")
    ap.add_argument("-q", "--query-file", default="query.q",
        help="Path to Stratego .q query file")
    ap.add_argument("-v", "--verifyta-path", default=ENV_VERIFYTA_PATH, help=
        "Path to verifyta executable")

    args = ap.parse_args()

    base_path = os.path.dirname(os.path.realpath(__file__)) 
    template_file = os.path.join(base_path, args.template_file)
    query_file = os.path.join(base_path, args.query_file)
    time.sleep(5)
    run(template_file, query_file, args.verifyta_path)
    shutdown_rclpy()