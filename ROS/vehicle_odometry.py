import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry

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

