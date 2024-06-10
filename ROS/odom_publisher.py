import math

from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import rclpy
import numpy as np
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster, TransformListener, Buffer


def quaternion_from_euler(roll, pitch, yaw):
    roll /= 2.0
    pitch /= 2.0
    yaw /= 2.0
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cc = cr*cy
    cs = cr*sy
    sc = sr*cy
    ss = sr*sy

    q = np.empty((4, ))
    q[0] = cp*sc - sp*cs
    q[1] = cp*ss + sp*cc
    q[2] = cp*cs - sp*sc
    q[3] = cp*cc + sp*ss

    return q





class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')
        # Declare and acquire `turtlename` parameter

        # Initialize the transform broadcaster
        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer,self)
        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )
        self.time = Time()
        self.msg = None
        self.yaw = 0.0
        self.yaw_stompc = 0.0
        self.local_position_msg = None
        self.attitude_q = None
        self.odom_pub = self.create_publisher(Odometry, "odom", qos_profile)
        self.subscription = self.create_subscription(
            VehicleOdometry,
           '/fmu/out/vehicle_odometry',
            self.handle_turtle_pose,
            qos_profile)
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        qos_profile_clock = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )
        self.vehicle_command_publisher = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos_profile_clock)

        self.timer = self.create_timer(0.05, self.send_transform)

    def vehicle_local_position_callback(self, msg):
        self.local_position_msg = msg
        self.yaw = msg.heading

    def vehicle_attitude_callback(self, msg):
        self.attitude_q = msg.q
    def clock_callback (self, msg):
        self.time = msg.clock


    def send_transform(self):
        if(self.msg == None): return
        # Read message content an d assign it to


        

        current_time = self.time
       
        map_odom_frame = TransformStamped()
        map_odom_frame.header.stamp = current_time
        map_odom_frame.header.frame_id = "map"
        map_odom_frame.child_frame_id = "odom"

        map_odom_frame.transform.translation.x = 0.0
        map_odom_frame.transform.translation.y = 0.0
        map_odom_frame.transform.translation.z = 0.0

        map_odom_frame.transform.rotation.x = 0.0
        map_odom_frame.transform.rotation.y = 0.0
        map_odom_frame.transform.rotation.z = 0.0
        map_odom_frame.transform.rotation.w = 1.0


        odom_base_link_frame= TransformStamped()
        odom_base_link_frame.header.stamp = current_time
        odom_base_link_frame.header.frame_id = "odom"
        odom_base_link_frame.child_frame_id = "base_link"


        odom_base_link_frame.transform.translation.x = float(self.local_position_msg.x)
        odom_base_link_frame.transform.translation.y = float(self.local_position_msg.y * -1)
        odom_base_link_frame.transform.translation.z = 0.0

        q = quaternion_from_euler(0.,0.,self.yaw * -1)
        odom_base_link_frame.transform.rotation.x = float(self.attitude_q[1])
        odom_base_link_frame.transform.rotation.y = float(self.attitude_q[2]) * -1
        odom_base_link_frame.transform.rotation.z = float(self.attitude_q[3]) * -1
        odom_base_link_frame.transform.rotation.w = float(self.attitude_q[0]) 

        #odom_base_link_frame.transform.rotation.x = 0.0    
        #odom_base_link_frame.transform.rotation.y = 0.0
        #odom_base_link_frame.transform.rotation.z = 0.0
        #odom_base_link_frame.transform.rotation.w = float(self.msg.position[3])
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        # first, we'll publish the transform over tf
        
   
       

        base_link_base_footprint_frame = TransformStamped()
        base_link_base_footprint_frame.header.stamp = current_time
        base_link_base_footprint_frame.header.frame_id = "base_link"
        base_link_base_footprint_frame.child_frame_id = "base_footprint"


        base_link_base_footprint_frame.transform.translation.x = 0.0
        base_link_base_footprint_frame.transform.translation.y = 0.0
        base_link_base_footprint_frame.transform.translation.z = 0.0

        base_link_base_footprint_frame.transform.rotation.x = 0.0
        base_link_base_footprint_frame.transform.rotation.y = 0.0
        base_link_base_footprint_frame.transform.rotation.z = 0.0
        base_link_base_footprint_frame.transform.rotation.w = 1.0


        base_link_to_sensor_lidar = TransformStamped()
        base_link_to_sensor_lidar.header.stamp = current_time
        base_link_to_sensor_lidar.header.frame_id = "base_footprint"
        base_link_to_sensor_lidar.child_frame_id = "x500_depth_0/OakD-Lite/base_link/StereoOV7251"


        base_link_to_sensor_lidar.transform.translation.x = 0.0
        base_link_to_sensor_lidar.transform.translation.y = 0.0
        base_link_to_sensor_lidar.transform.translation.z = 0.0

        base_link_to_sensor_lidar.transform.rotation.x = 0.0
        base_link_to_sensor_lidar.transform.rotation.y = 0.0
        base_link_to_sensor_lidar.transform.rotation.z = 0.0
        base_link_to_sensor_lidar.transform.rotation.w = 1.0


        

        self.tf_broadcaster.sendTransform([base_link_to_sensor_lidar, odom_base_link_frame, base_link_base_footprint_frame])
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        # first, we'll publish the transform over tf
        #self.tf_broadcaster.sendTransform(test)
        # next, we'll publish the odometry message over ROS
        odom = Odometry()

        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "x500_depth_0"


        p = Point()
        p.x = float(self.msg.position[0])
        p.y = float(self.msg.position[1])
        p.z = float(self.msg.position[2])

        # set the position
        odom.pose.pose.position = p

        q_msg = Quaternion()
        q_msg.x = q[0]
        q_msg.y = q[1]
        q_msg.z = q[2]
        q_msg.w = q[3]
        odom.pose.pose.orientation = q_msg

        # set the velocity
        v_linear = Vector3()
        v_angular = Vector3()

        v_linear.x = float(self.msg.velocity[0])
        v_linear.y = float(self.msg.velocity[1])
        v_linear.z = float(self.msg.velocity[2])

        v_angular.x = float(self.msg.angular_velocity[0])
        v_angular.y = float(self.msg.angular_velocity[1])
        v_angular.z = float(self.msg.angular_velocity[2])

        twist = Twist()
        twist.linear = v_linear
        twist.angular = v_angular

        odom.twist.twist = twist
    
        # publish the message
        # self.odom_pub.publish(odom)

    def handle_turtle_pose(self, msg):
        self.msg = msg
        





def init(odom_publisher_instance, args=None) -> None:
    print('Starting odom publisher node...')
   

    def run_odom_publisher():
       executor = rclpy.executors.SingleThreadedExecutor()
       executor.add_node(odom_publisher_instance)
       executor.spin()
    offboard_thread = threading.Thread(target=run_odom_publisher)
    offboard_thread.start()
    
