from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'   
        ),
  
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in',  '/cloud'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.1,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.02,
                'range_min': 0.45,
                'range_max': 20.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'concurrency_level':0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
