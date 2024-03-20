import random
import argparse
import os
import rclpy
import sys
import threading
import strategoutil as sutil
import time

sys.path.insert(0, '../')
from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor, odom_publisher
import time
from model_interface import QueueLengthController
from environment import generate_environment, build_uppaal_environment_array_string, unpack_environment
global offboard_control_instance
global odom_publisher_instance
INITIAL_X = 0.0
INITIAL_Y = 0.0
e = 0.2
uppaa_e = 0.5
def activate_action(x, y, yaw):
      
    drone_x = x
    drone_y = y
    offboard_control_instance.x = drone_x
    offboard_control_instance.y = drone_y
    offboard_control_instance.yaw = yaw
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())
    odom_publisher_instance.yaw_stompc = yaw

    time.sleep(2)
    while((drone_x-e > curr_x or curr_x > drone_x+e) or (drone_y-e > curr_y or curr_y > drone_y+e)):
        time.sleep(0.5)
        curr_x = float(vehicle_odometry.get_drone_pos_x())
        curr_y = float(vehicle_odometry.get_drone_pos_y())

    curr_avg_distance = lidar_sensor.get_avg_distance()
    print("Distance: ",curr_avg_distance)

    return curr_x,curr_y,yaw,curr_avg_distance

def calculate_safe_states(seen_x, seen_y, seen_distances, seen_yaw, x,y,yaw,distance, N):
    if(yaw == 3.14 or yaw == -3.14):
        #-x
        while(distance > 2.0):
            distance -= uppaa_e
            x-=uppaa_e
            seen_x.append(x)
            seen_y.append(y)
            seen_yaw.append(yaw)
            seen_distances.append(distance)
            N+=1
        
    elif(yaw == 0):
        #+x
        while(distance > 2.0):
            distance -= uppaa_e
            x+=uppaa_e
            seen_x.append(x)
            seen_y.append(y)
            seen_yaw.append(yaw)
            seen_distances.append(distance)
            N+=1


    elif(yaw == -1.57):
        #-y
        while(distance > 2.0):
            distance -= uppaa_e
            y-=uppaa_e
            seen_x.append(x)
            seen_y.append(y)
            seen_yaw.append(yaw)
            seen_distances.append(distance)
            N+=1

    elif(yaw == 1.57):
        #+y
        while(distance > 2.0):
            distance -= uppaa_e
            y+=uppaa_e
            seen_x.append(x)
            seen_y.append(y)
            seen_yaw.append(yaw)
            seen_distances.append(distance)
            N+=1

    return seen_x, seen_y, seen_yaw, seen_distances, N


def run(template_file, query_file, verifyta_path):
    print("running uppaal")
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "goal_x", "goal_y", "avg_distance", "yaw", "NLOOP", "seen_x", "seen_y", "seen_yaw", "seen_distance", "NX", "NY", "NYAW", "NDISTANCE", "environment"])
    # initial drone state
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = 0.0
    seen_x = []
    seen_y = []
    seen_yaw = []
    seen_distance = []
    avg_distance = lidar_sensor.get_avg_distance()
    N = 0

    environment = generate_environment()
    
    controller.generate_query_file(state_vars=["DroneController.DescisionState", unpack_environment(environment, "environment")], 
                                   point_vars=["x", "y", "yaw"], 
                                   observables=["action", "x", "y", "yaw", "current_step_length"])

    goal_x = 0
    goal_y = -9.5
    L = 1000 # simulation length
    K = 1  # every K we will do MPC
    for k in range(L):
        K_START_TIME = time.time()
        # run plant

        #handle_action(next_action);
        
        #<- readings fra diverse sensor
        x,y,yaw,avg_distance = activate_action(x,y,yaw)
        seen_x.append(x)
        seen_y.append(y)
        seen_yaw.append(yaw)
        seen_distance.append(avg_distance)
        N = N + 1
        seen_x, seen_y, seen_yaw, seen_distance, N = calculate_safe_states(seen_x, seen_y, seen_distance, seen_yaw, x, y, yaw, avg_distance, N)
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
                "NLOOP_PS":N,
                "NLOOP": N,
                "NY": N,
                "NX": N,
                "NYAW": N,
                "NDISTANCE": N,
                "seen_x": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_x][::-1]) + "]"),
                "seen_y": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_y][::-1]) + "]"),
                "seen_yaw": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_yaw][::-1]) + "]"),
                "seen_distance": sutil.array_to_stratego("[" + ','.join([str(x) for x in seen_distance][::-1]) + "]"),
                "environment": build_uppaal_environment_array_string(environment)
                
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
