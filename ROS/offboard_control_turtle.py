import rclpy
import threading
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
class OffboardControlTurtle(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Create subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile)


        # Initialize variables
        self.distance_moved = 0.0
        self.initial_position = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw =math.atan2(2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z, msg.pose.pose.orientation.w*msg.pose.pose.orientation.w - msg.pose.pose.orientation.z*msg.pose.pose.orientation.z)
        print(self.yaw)
        if self.initial_position is None:
            self.initial_position = (self.x, self.y)
        else:
            self.distance_moved = math.sqrt((self.x - self.initial_position[0])**2 + (self.y - self.initial_position[1])**2)

    
    def move_robot(self, distance, yaw_amount, velocity):
        command = Twist()
        print(yaw_amount)
        if distance != None:
            command.linear.x = velocity
            self.vel_publisher.publish(command)
            while self.distance_moved <= distance:
                command.linear.x = 0.0

        if yaw_amount != None:
            command.angular.z = -velocity
            self.vel_publisher.publish(command)
            while((self.yaw - 0.01 > yaw_amount or yaw_amount > self.yaw + 0.01)):
                command.angular.z = 0.0

        
        self.vel_publisher.publish(command)
        self.initial_position = None
        self.distance_moved = 0.0

  

def init(offboard_control_instance, args=None) -> None:
    print('Starting offboard control node...')
   

    def run_offboard_control():
       rclpy.spin(offboard_control_instance)
       rclpy.shutdown()
    offboard_thread = threading.Thread(target=run_offboard_control)
    offboard_thread.start()
    


