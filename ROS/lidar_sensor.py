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

        # If we are to divide width into 3 columns, we are getting 640 / 3 = 213.3333, we have to add -1 so that we have an even split
        # width:  639 / 3 = 213, with one rest
        # Height: 480 / 3 = 160, non rest
        SPLIT_SIZE = 3
        N_ZONES = SPLIT_SIZE**2


        # Split height into SPLIT_SIZE, check if there is an uneven number of points and append rest points to the middle zone
        # Does the same for width under.
        HEIGHT_SPLIT_REST = msg.height % SPLIT_SIZE
        HEIGHT_SPLIT_SIZE = (msg.height - HEIGHT_SPLIT_REST) / SPLIT_SIZE
        print("hsr: {}, hss: {}".format(HEIGHT_SPLIT_REST,HEIGHT_SPLIT_SIZE))
        #HEIGHT_SPLITS = [HEIGHT_SPLIT for _ in range(SPLIT_SIZE)]

        #if HEIGHT_SPLIT_REST is not 0:
        #    HEIGHT_SPLITS[SPLIT_SIZE//2] += HEIGHT_SPLIT_REST

        WIDTH_SPLIT_REST = msg.width % SPLIT_SIZE
        WIDTH_SPLIT_SIZE = (msg.width - WIDTH_SPLIT_REST) / SPLIT_SIZE
        print("wsr: {}, wss: {}".format(WIDTH_SPLIT_REST,WIDTH_SPLIT_SIZE))
        #WIDTH_SPLITS = [WIDTH_SPLIT_SIZE for _ in range(SPLIT_SIZE)]

        #if WIDTH_SPLIT_REST is not 0:
        #    WIDTH_SPLITS[SPLIT_SIZE//2] += WIDTH_SPLIT_REST

        HEIGHT_RANGES = []
        WIDTH_RANGES = []

        height_start_range = 0
        width_start_range = 0
        for i in range(SPLIT_SIZE):
            height_end_range = (height_start_range-1) + HEIGHT_SPLIT_SIZE
            width_end_range = (width_start_range-1) + WIDTH_SPLIT_SIZE
            if i == SPLIT_SIZE // 2:
                height_end_range += HEIGHT_SPLIT_REST
                width_end_range += WIDTH_SPLIT_REST

            HEIGHT_RANGES.append((int(height_start_range),int(height_end_range)))
            WIDTH_RANGES.append((int(width_start_range),int(width_end_range)))

            height_start_range += HEIGHT_SPLIT_SIZE
            width_start_range += WIDTH_SPLIT_SIZE

            if i == SPLIT_SIZE // 2:
                height_start_range += HEIGHT_SPLIT_REST
                width_start_range += WIDTH_SPLIT_REST
    
        print(len(HEIGHT_RANGES),HEIGHT_RANGES)
        print(len(WIDTH_RANGES),WIDTH_RANGES)

        distances = []
        for h_start,h_end in HEIGHT_RANGES:
            points_to_get_test = []
            for w_start,w_end in WIDTH_RANGES:
                for h in range(h_start,h_end+1):
                    points_to_get_test.extend([(h,w) for w in range(w_start,w_end+1)])
            distances.extend(points_to_get_test)
            print(len(points_to_get_test))

        print("Distances",len(distances))

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



