import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry
from sensor_msgs.msg import PointCloud2
from ROS import point_reader

class LidarSensorListener(Node):
    def __init__(self) -> None:
        super().__init__('vehicle_odom')
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10000
        )   
        self.zone_points = []

        # SPLIT_SIZE is used to indicate how many zones that are going to be made. The number of zones created is SPLIT_SIZE^2
        self.SPLIT_SIZE = 7
        # Create subscribers
        self.lidar_sensor_subscriber = self.create_subscription(
            PointCloud2, '/cloud', self.lidar_sensor_callback, 10)
    
      
    def lidar_sensor_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber."""
        #self.x = vehicle_local_position.position[0]
        #self.y = vehicle_local_position.position[1]

        # If we are to divide width into 3 columns, we are getting 640 / 3 = 213.3333, we have to add -1 so that we have an even split
        # width:  639 / 3 = 213, with one rest
        # Height: 480 / 3 = 160, non rest
        # SPLIT_SIZE is used to indicate how many zones that are going to be made. The number of zones created is SPLIT_SIZE^2
        N_ZONES = self.SPLIT_SIZE**2


        # Split height into SPLIT_SIZE, check if there is an uneven number of points and append rest points to the middle zone
        # Does the same for width under.
        HEIGHT_SPLIT_REST = msg.height % self.SPLIT_SIZE
        HEIGHT_SPLIT_SIZE = (msg.height - HEIGHT_SPLIT_REST) / self.SPLIT_SIZE

        WIDTH_SPLIT_REST = msg.width % self.SPLIT_SIZE
        WIDTH_SPLIT_SIZE = (msg.width - WIDTH_SPLIT_REST) / self.SPLIT_SIZE

        HEIGHT_RANGES = []
        WIDTH_RANGES = []

        height_start_range = 0
        width_start_range = 0
        for i in range(self.SPLIT_SIZE):
            height_end_range = (height_start_range-1) + HEIGHT_SPLIT_SIZE
            width_end_range = (width_start_range-1) + WIDTH_SPLIT_SIZE
            if i == self.SPLIT_SIZE // 2:
                height_end_range += HEIGHT_SPLIT_REST
                width_end_range += WIDTH_SPLIT_REST

            HEIGHT_RANGES.append((int(height_start_range),int(height_end_range)))
            WIDTH_RANGES.append((int(width_start_range),int(width_end_range)))

            height_start_range += HEIGHT_SPLIT_SIZE
            width_start_range += WIDTH_SPLIT_SIZE

            if i == self.SPLIT_SIZE // 2:
                height_start_range += HEIGHT_SPLIT_REST
                width_start_range += WIDTH_SPLIT_REST
    
        zones = []
        for w_start,w_end in WIDTH_RANGES:
            for h_start,h_end in HEIGHT_RANGES:
                points_to_get_test = []
                for w in range(w_start,w_end+1):
                    for h in range(h_start,h_end+1):
                        points_to_get_test.append((w,h))
                zones.append(points_to_get_test)

        for zone in zones:
                generator = point_reader.read_points(msg, skip_nans=True, field_names=("x,y,z"), uvs=zone)
                points = list(generator)
                self.zone_points.append(points)

        self.destroy_node()


def get_avg_distance():
    zones,SPLIT_SIZE = get_points()
    MIDDLE_ZONE_SPREAD = SPLIT_SIZE//2   # Gives us the middle zone in a row of the SPLIT_SIZE**2 matrix
    MIDDLE_ROW = SPLIT_SIZE**2//2 # Gives us the middle row in the matrix
    
    avg_zone_dists = []
    for zone in zones[MIDDLE_ROW-MIDDLE_ZONE_SPREAD:MIDDLE_ROW+MIDDLE_ZONE_SPREAD+1]:
        if zone == None:
            avg_zone_dists.append(2.0)
            continue
        zone_points = [tpl for tpl in zone if not any(math.isinf(d) for d in tpl)]
        if len(zone_points) == 0:
            print(f'zone with 0 non inf points: {zone}')
            continue
        print("zone_points: {}".format(zone_points[:5]))

        avg_zone_dist = sum([x for x,_,_ in zone_points]) / len(zone_points)
        avg_zone_dists.append(avg_zone_dist)

    #TODO: Skal det laves sådan at det ikke er split_size den tager efter her, siden der kan være færre zoner end 
    #      forventet pga. det med nogle zoner er fuld af inf?
    if SPLIT_SIZE % 2 == 0:
        return_dist = (avg_zone_dists[(SPLIT_SIZE//2)-1] + avg_zone_dists[SPLIT_SIZE//2]) / 2
    else:
        return_dist = avg_zone_dists[SPLIT_SIZE//2]

    print("Test print:",avg_zone_dists[1:-1])
    if any(x < 2 for x in avg_zone_dists[1:-1]):
        return_dist = min(avg_zone_dists)

    return return_dist

def get_points():
    executor = rclpy.executors.SingleThreadedExecutor()
    vehicle_listener = LidarSensorListener()
    executor.add_node(vehicle_listener)
    executor.spin_once()
    return vehicle_listener.zone_points, vehicle_listener.SPLIT_SIZE



