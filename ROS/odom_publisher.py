import math

from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import rclpy
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformListener, Buffer
from turtlesim.msg import Pose
import ctypes

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q





class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')
        print("init odom publisher")
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
        self.odom_pub = self.create_publisher(Odometry, "odom", qos_profile)
        self.subscription = self.create_subscription(
            VehicleOdometry,
           '/fmu/out/vehicle_odometry',
            self.handle_turtle_pose,
            qos_profile)
        

        qos_profile_clock = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
            depth=1
        )
        self.vehicle_command_publisher = self.create_subscription(
            Clock, '/clock', self.clock_callback, qos_profile_clock)

        self.timer = self.create_timer(0.1, self.send_transform)
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


        odom_base_link_frame.transform.translation.x = float(self.msg.position[0])
        odom_base_link_frame.transform.translation.y = float(self.msg.position[1])
        odom_base_link_frame.transform.translation.z = 0.0

        q = quaternion_from_euler(float(self.msg.q[0]),float(self.msg.q[1]),float(self.msg.q[2]))

        odom_base_link_frame.transform.rotation.x = float(self.msg.q[0])
        odom_base_link_frame.transform.rotation.y = float(self.msg.q[1]) * -1
        odom_base_link_frame.transform.rotation.z = float(self.msg.q[2]) * -1
        odom_base_link_frame.transform.rotation.w = float(self.msg.q[3]) 

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

        self.tf_broadcaster.sendTransform([map_odom_frame, base_link_to_sensor_lidar, odom_base_link_frame, base_link_base_footprint_frame])
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
        self.odom_pub.publish(odom)

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
    
