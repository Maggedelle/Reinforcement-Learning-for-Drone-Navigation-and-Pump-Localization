import rclpy
import threading
import time

from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock

class VehicleOdomListener(Node):
    """Node for controlling a vehicle in offboard mode."""
    x = 0.0
    y = 0.0
    def __init__(self) -> None:
        super().__init__('vehicle_odom')
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )   
        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
    
      
    def vehicle_odometry_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.x = vehicle_local_position.position[0]
        self.y = vehicle_local_position.position[1]
        self.destroy_node()

def get_drone_pos_x():
    executor = rclpy.executors.SingleThreadedExecutor()
    vehicle_listener = VehicleOdomListener()
    executor.add_node(vehicle_listener)
    executor.spin_once()
    return str(vehicle_listener.x)

def get_drone_pos_y():
    executor = rclpy.executors.SingleThreadedExecutor()
    vehicle_listener = VehicleOdomListener()
    executor.add_node(vehicle_listener)
    executor.spin_once()
    return str(vehicle_listener.y)

class MapDroneFrameListener(Node):
    def __init__(self):
        super().__init__('map_drone_tf2_frame_listener')

        self.target_frame = self.declare_parameter('target_frame', 'odom').get_parameter_value().string_value
        self.transform = TransformStamped()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.5,self.get_tf)

        self.time = Time()
        qos_profile_clock = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )

        self.sim_clock_subscriber = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos_profile_clock)
        
    def clock_callback (self, msg):
        self.time = msg.clock


    def get_tf(self):
        from_frame = self.target_frame
        to_frame = 'base_link'
        curr_time = self.time

        try:
            self.transform = self.tf_buffer.lookup_transform(to_frame,from_frame,curr_time)
        except Exception as e:
            return
        
def init_map_drone_tf(map_drone_frame_listener_instance, args=None) -> None:
    print('Starting map / drone tf2 listener node...')


    def run_tf_listener():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(map_drone_frame_listener_instance)
        executor.spin()
    tf2_thread = threading.Thread(target=run_tf_listener)
    tf2_thread.start()