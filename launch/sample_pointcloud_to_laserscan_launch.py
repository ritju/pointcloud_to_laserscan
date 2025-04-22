from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

if 'BACK_DEPTH_MIN_HEIGHT' and 'BACK_DEPTH_MAX_HEIGHT' in os.environ:
    back_depth_min_height = float(os.environ.get('BACK_DEPTH_MIN_HEIGHT'))
    back_depth_max_height = float(os.environ.get('BACK_DEPTH_MAX_HEIGHT'))
else:
    back_depth_min_height = -0.927
    back_depth_max_height = 1.0

if 'FRONT_DOWN_DEPTH_MIN_HEIGHT' and 'FRONT_DOWN_DEPTH_MAX_HEIGHT' in os.environ:
    front_down_depth_min_height = float(os.environ.get('FRONT_DOWN_DEPTH_MIN_HEIGHT'))
    front_down_depth_max_height = float(os.environ.get('FRONT_DOWN_DEPTH_MAX_HEIGHT'))
else:
    front_down_depth_min_height = -0.41
    front_down_depth_max_height = 1.0

if 'FRONT_UP_DEPTH_MIN_HEIGHT' and 'FRONT_UP_DEPTH_MAX_HEIGHT' in os.environ:
    front_up_depth_min_height = float(os.environ.get('FRONT_UP_DEPTH_MIN_HEIGHT'))
    front_up_depth_max_height = float(os.environ.get('FRONT_UP_DEPTH_MAX_HEIGHT'))
else:
    front_up_depth_min_height = -0.4
    front_up_depth_max_height = 1.8


parameters_back = [{
        'target_frame': 'camera3_color_frame',
        'transform_tolerance': 0.01,
        'min_height': -0.927,
        'max_height': 1.0,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 10.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

parameters_front_up = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': -0.40,
        'max_height': 1.8,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 10.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

parameters_front_down = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': -0.41,
        'max_height': 1.0,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 10.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

node_back = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', 'camera3/depth/points'),
                ('scan', 'point_scan_back')],
    parameters=parameters_back,
    name='pointcloud_to_laserscan_back'
    )

node_front_up = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', 'camera1/depth/points'),
                ('scan', 'point_scan_front_up')],
    parameters=parameters_front_up,
    name='pointcloud_to_laserscan_front_up'
    )

node_front_down = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', 'camera2/depth/points'),
                ('scan', 'point_scan_front_down')],
    parameters=parameters_front_down,
    name='pointcloud_to_laserscan_front_down'
    )

def generate_launch_description():
    
    return LaunchDescription([
        node_back,
        node_front_up,
        node_front_down,   
    ])
