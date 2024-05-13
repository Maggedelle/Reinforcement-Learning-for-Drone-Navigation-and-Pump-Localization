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
map_config = get_baseline_one_pump_config()
ENV_VERIFYTA_PATH = os.environ['VERIFYTA_PATH']

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
            yaw_plus_half_pi = offboard_control_instance.yaw + half_PI_left
            yaw_plus_half_pi = (yaw_plus_half_pi + math.pi) % (2 * math.pi) - math.pi
            radians = yaw_plus_half_pi
            vel = -0.4
        else: 
            yaw_plus_half_pi = offboard_control_instance.yaw + half_PI_right
            yaw_plus_half_pi = (yaw_plus_half_pi + math.pi) % (2 * math.pi) - math.pi
            radians = yaw_plus_half_pi
            vel = 0.4
        print(radians, offboard_control_instance.yaw)
        offboard_control_instance.move_robot(None, radians, vel)
        while(offboard_control_instance.is_running_action == True):
            time.sleep(0.1)
        print("ran one action")



    

def run(template_file, query_file, verifyta_path):
    offboard_control_instance.move_robot(None, None, 0.01)
    controller = QueueLengthController(
    templatefile=template_file,
    state_names=["x", "y", "yaw", "width_map","height_map", "map", "granularity_map", "open", "discovery_reward", "turning_cost", "moving_cost", "drone_diameter", "safety_range", "range_laser", "laser_range_diameter", "pump_exploration_reward"])
    # initial drone state
    x = offboard_control_instance.x
    y = offboard_control_instance.y

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

    #offboard_control_instance.move_robot(None, 1.0, 0.2)
    
    run_action_seq([4,5,4,5])

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