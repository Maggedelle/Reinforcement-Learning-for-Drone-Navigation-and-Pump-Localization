from flask import Flask, request
from ROS import vehicle_odometry, offboard_control, camera_control
import rclpy
import os
import threading
import time



app = Flask(__name__)
global offboard_control_instance

global queue
queue = []

global running_task
running_task = False

@app.route('/')
def hello():
    return '0'
    
@app.route('/get_position_x')
def get_postion_x():
    return vehicle_odometry.get_drone_pos_x()

@app.route('/get_position_y')
def get_postion_y():
    return vehicle_odometry.get_drone_pos_y()

@app.route('/shutdown_drone')
def shutdown_drone():
    queue.append({"type":"shutdown_drone"})
    return 'Shutting down drone'

global image_num
image_num = 1

@app.route('/take_image')
def take_image():
    global image_num
    distance = str(1.65 - offboard_control_instance.x)
    if image_num % 6 == 0:
        image_num = 1
    camera_control.take_image(distance,image_num)
    image_num += 1
    return "Image taken"


@app.route('/move_drone')
def move_drone_endpoint():
    new_x = request.args.get('x')
    new_y = request.args.get('y')
    print(new_x,offboard_control_instance.x,new_y,offboard_control_instance.y)
    queue.append({"type":"move_drone", "x": float(new_x), "y": float(new_y)})
    return 'moving drone along x axis'

def init_image_bridge():
    print("Starting image bridge...")
    def run_bridge():
        print("image bridge started...")
        os.system('ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image')
    image_bridge_thread = threading.Thread(target=run_bridge)
    image_bridge_thread.start()


def move_drone(task):
    print("Beginning to move drone")
    global running_task
    e = 0.03
    offboard_control_instance.x = task["x"]
    offboard_control_instance.y = task["y"]
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())
    while((task["x"]-e > curr_x or curr_x > task["x"]+e) or (task["y"]-e > curr_y or curr_y > task["y"]+e)):
        time.sleep(0.5)
        curr_x = float(vehicle_odometry.get_drone_pos_x())
        curr_y = float(vehicle_odometry.get_drone_pos_y())
    running_task = False

        

    
        
    

def main_loop():
    threading.Timer(0.1, main_loop).start()
    global queue
    global running_task

    if(len(queue) == 0 or running_task == True):
        return 0
    
    running_task = True
    current_task = queue.pop(0)
    match current_task["type"]:
        case "move_drone":
            move_drone(current_task)
        case "shutdown_drone":
            if offboard_control_instance != None:
                offboard_control_instance.shutdown_drone = True
                running_task = False
        
    


def init_rclpy():
    print("initializing rclpy")
    rclpy.init()


if __name__ == "__main__":
    init_rclpy()
    offboard_control_instance = offboard_control.OffboardControl()
    offboard_control.init(offboard_control_instance)
    init_image_bridge()
    main_loop()
    app.run()

