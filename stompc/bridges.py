import os
import threading
import rclpy
import sys

sys.path.insert(0, '../')
from dotenv import load_dotenv
load_dotenv()

ENV_DOMAIN = os.environ['DOMAIN']


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
