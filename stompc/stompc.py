import random
import argparse
import os
import rclpy
import sys
import threading
import strategoutil as sutil

sys.path.insert(0, '../')
from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor
import time
from model_interface import QueueLengthController

global offboard_control_instance
INITIAL_X = 0.0
INITIAL_Y = 0.0
e = 0.2






def activate_action(x, y, yaw):
      
    drone_x = x
    drone_y = y
    offboard_control_instance.x = drone_x
    offboard_control_instance.y = drone_y
    offboard_control_instance.yaw = yaw
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())

    time.sleep(2)
    while((drone_x-e > curr_x or curr_x > drone_x+e) or (drone_y-e > curr_y or curr_y > drone_y+e)):
        time.sleep(0.5)
        curr_x = float(vehicle_odometry.get_drone_pos_x())
        curr_y = float(vehicle_odometry.get_drone_pos_y())

    curr_avg_distance = lidar_sensor.get_avg_distance()
    print("Distance: ",curr_avg_distance)
    return curr_x,curr_y,yaw,curr_avg_distance

def run(template_file, query_file, verifyta_path):
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "goal_x", "goal_y", "avg_distance", "yaw", "NLOOP", "seen_x", "seen_y", "seen_yaw", "seen_distance", "NX", "NY", "NYAW", "NDISTANCE"])
    # initial drone state
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = 0.0
    seen_x = []
    seen_y = []
    seen_yaw = []
    seen_distance = []
    N = 0

    avg_distance = lidar_sensor.get_avg_distance()

    goal_x = 0
    goal_y = -9.5
    L = 1000 # simulation length
    K = 1  # every K we will do MPC
    for k in range(L):
        # run plant

        #handle_action(next_action);
        
        #<- readings fra diverse sensor
        x,y,yaw,avg_distance = activate_action(x,y,yaw)
        seen_x.append(x)
        seen_y.append(y)
        seen_yaw.append(yaw)
        seen_distance.append(avg_distance)
        N = N + 1
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
                "yaw":  yaw,
                "avg_distance":avg_distance,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "NLOOP": N,
                "NY": N,
                "NX": N,
                "NYAW": N,
                "NDISTANCE": N,
                "seen_x": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_x]) + "]"),
                "seen_y": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_y]) + "]"),
                "seen_yaw": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_yaw]) + "]"),
                "seen_distance": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_distance]) + "]")
                
            }
            #print(state)
            controller.insert_state(state)
            action, x,y, yaw = controller.run(
                queryfile=query_file,
                verifyta_path=verifyta_path)
            print(action,x,y,yaw)
            
            #print("actions:",action_seq)


def init_image_bridge():
    print("Starting image bridge...")
    def run_bridge():
        print("image bridge started...")
        os.system('ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image')
    image_bridge_thread = threading.Thread(target=run_bridge)
    image_bridge_thread.start()

def init_rclpy():
    print("initializing rclpy")
    rclpy.init(domain_id=2)

if __name__ == "__main__":
    
    init_rclpy()
    offboard_control_instance = offboard_control.OffboardControl()
    offboard_control.init(offboard_control_instance)
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
    time.sleep(5)
    run(template_file, query_file, args.verifyta_path)
