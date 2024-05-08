import random
import argparse
import os
import rclpy
import sys
import threading
import strategoutil as sutil
sys.path.insert(0, '../')

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

ENV_VERIFYTA_PATH = os.environ['VERIFYTA_PATH']

def run(template_file, query_file, verifyta_path):
    offboard_control_instance.move_robot(None, 0.0)

if __name__ == "__main__":
    init_rclpy()

    offboard_control_instance = offboard_control_turtle.OffboardControlTurtle()
    offboard_control_turtle.init(offboard_control_instance)

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
    time.sleep(5)
    run(template_file, query_file, args.verifyta_path)
    shutdown_rclpy()