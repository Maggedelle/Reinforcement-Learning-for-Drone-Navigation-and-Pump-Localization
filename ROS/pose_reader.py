import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from tf2_msgs.msg import TFMessage

class PoseReaderListener(Node):
    """Node for controlling a vehicle in offboard mode."""
    x = 0.0
    y = 0.0
    def __init__(self) -> None:
        super().__init__('vehicle_odom')
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )   
        # Create subscribers
        self.pose_reader_subscriber = self.create_subscription(
            TFMessage, '/world/default/pose/info', self.pose_reader_callback, qos_profile)
    
      
    def pose_reader_callback(self, tf_message):
        """Callback function for vehicle_local_position topic subscriber."""
        self.x = tf_message.transforms[2].transform.translation.x
        self.y = tf_message.transforms[2].transform.translation.y
        self.destroy_node()

def get_pose():
    executor = rclpy.executors.SingleThreadedExecutor()
    pose_listener = PoseReaderListener()
    executor.add_node(pose_listener)
    executor.spin_once()
    return (pose_listener.x, pose_listener.y)


