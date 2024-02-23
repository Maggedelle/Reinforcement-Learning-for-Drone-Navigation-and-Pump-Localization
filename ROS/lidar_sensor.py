import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from sensor_msgs.msg import PointCloud2
from ROS import point_reader

class LidarSensorListener(Node):
    """Node for controlling a vehicle in offboard mode."""
    x = 0.0
    y = 0.0
    points = None
    width = None
    height = None
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
        self.lidar_sensor_subscriber = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.lidar_sensor_callback, 10)
    
      
    def lidar_sensor_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber."""
        #self.x = vehicle_local_position.position[0]
        #self.y = vehicle_local_position.position[1]

        RANGE_TO_COVER = 50

        points_to_get = []
        self.width = msg.width
        self.height = msg.height
        print("width: {}, height: {}".format(self.width,self.height))
        width_mid = self.width // 2
        height_mid = self.height // 2

        # We want to get the middle sqaure of points
        for i in range(height_mid-RANGE_TO_COVER, height_mid+RANGE_TO_COVER):
            for j in range(width_mid-RANGE_TO_COVER,width_mid+RANGE_TO_COVER):
                points_to_get.append((i,j))
        
        generator = point_reader.read_points(msg, skip_nans=True, field_names=("x", "y", "z"), uvs=points_to_get)
        self.points = list(generator)
        self.destroy_node()


def get_avg_distance():
    points = get_points()
    
    if points == None:
        return 2.0
    
    points = [x for x,_,_ in points if not math.isinf(x)]
    dist_avg = sum(points)/len(points)
    #for i in range(height_mid-RANGE_TO_COVER, height_mid+RANGE_TO_COVER):
    #    for j in range(width_mid-RANGE_TO_COVER,width_mid+RANGE_TO_COVER):
    #        points_to_get.append((i,j))
            #x,y,z = points[i][j]
            #dist = ((x-drone_x)**2+(y-drone_y)**2)**0.5
            #dists.append(dist)

    return dist_avg



def get_points():
    executor = rclpy.executors.SingleThreadedExecutor()
    vehicle_listener = LidarSensorListener()
    executor.add_node(vehicle_listener)
    executor.spin_once()
    return vehicle_listener.points



