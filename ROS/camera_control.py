from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rclpy
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
bridge = CvBridge()

class ImageListener(Node):
    """Node for controlling a vehicle in offboard mode."""
    distance = 1.65
    image_num = 0
    
    def __init__(self, _distance, _image_num) -> None:
        super().__init__('vehicle_odom')

        self.distance = _distance
        self.image_num = _image_num
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )   
        # Create subscribers
        self.image_subscriber = self.create_subscription(
            Image, '/camera', self.image_callback, qos_profile)
    def image_callback(self, msg):
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(f'/home/sw9-bois/UPPAAL-TO-ROS-SIMULATION/pumpdetection_model/outputs/pics/camera_image_{str(self.distance)[:5]}_{str(self.image_num)}.jpg', cv2_img)
            self.destroy_node()

    
def take_image(distance, image_num):
    executor = rclpy.executors.SingleThreadedExecutor()
    image_listener = ImageListener(distance, image_num)
    executor.add_node(image_listener)
    executor.spin_once()
    return 0