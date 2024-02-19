import random
import argparse
import os
import rclpy
import sys
import threading
sys.path.insert(0, '../')
from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor
import time
from model_interface import QueueLengthController

global offboard_control_instance
global INITIAL_X,INITIAL_Y
INITIAL_X = 9
INITIAL_Y = 1


def activate_action(action,x, y, dist_to_object):
    match action:
        case 0:
            y-=1
        case 1:
            x+=1
        case 2:
            y+=1
        case 3:
            x-=1
        case _:
            print("unkown action")
            return x,y,dist_to_object

    drone_x = float(x - INITIAL_X)
    drone_y = float((y - INITIAL_Y) * -1)
    print("Beginning to move drone to {},{}".format(drone_x, drone_y))
    e = 0.03
    offboard_control_instance.x = drone_x
    offboard_control_instance.y = drone_y
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())
    while((drone_x-e > curr_x or curr_x > drone_x+e) or (drone_y-e > curr_y or curr_y > drone_y+e)):
        time.sleep(0.5)
        curr_x = float(vehicle_odometry.get_drone_pos_x())
        curr_y = float(vehicle_odometry.get_drone_pos_y())

    curr_dist_to_object = lidar_sensor.get_avg_distance()
    print(curr_dist_to_object)
    return x,y,curr_dist_to_object

def run(template_file, query_file, verifyta_path):

    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "goal_x", "goal_y"])

    # initial plant state
    x = INITIAL_X
    y = INITIAL_Y
    dist_to_object = lidar_sensor.get_avg_distance()
    print("initial distance: {}".format(dist_to_object))
    goal_x = 6.0
    goal_y = 8.0
    L = 30 # simulation length
    K = 1  # every K we will do MPC
    next_action = None
    for k in range(L):
        # run plant

        #handle_action(next_action);
        
        #<- readings fra diverse sensor
        x,y,dist_to_object = activate_action(next_action, x,y,dist_to_object)
        print(x,y)
        if x == goal_x and y == goal_y:
            print("found pump")
            break
        # report
        #print("Step: {}, x: {} cars, y: {} cars".format(k, x, y))

        if k % K == 0:
            # at each MPC step we want a clean template copy
            # to insert variables
            controller.init_simfile()
            
            # insert current state into simulation template
            state = {
                "x": x,
                "y": y,
                "goal_x": goal_x,
                "goal_y": goal_y
            }
            controller.insert_state(state)
            durations, action_seq = controller.run(
                queryfile=query_file,
                verifyta_path=verifyta_path)
            
            print(action_seq)

            next_action = action_seq[0]


def init_image_bridge():
    print("Starting image bridge...")
    def run_bridge():
        print("image bridge started...")
        os.system('ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image')
    image_bridge_thread = threading.Thread(target=run_bridge)
    image_bridge_thread.start()

def init_rclpy():
    print("initializing rclpy")
    rclpy.init()

if __name__ == "__main__":
    
    init_rclpy()
    offboard_control_instance = offboard_control.OffboardControl()
    offboard_control.init(offboard_control_instance)
    init_image_bridge()

    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--template-file", default="drone_model_stompc.xml", 
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
    
    run(template_file, query_file, args.verifyta_path)