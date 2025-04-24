from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

if 'BACK_DEPTH_MIN_HEIGHT' and 'BACK_DEPTH_MAX_HEIGHT' in os.environ:
    back_depth_min_height = float(os.environ.get('BACK_DEPTH_MIN_HEIGHT'))
    back_depth_max_height = float(os.environ.get('BACK_DEPTH_MAX_HEIGHT'))
else:
    back_depth_min_height = -0.35
    back_depth_max_height = 1.0

if 'FRONT_DEPTH_MIN_HEIGHT' and 'FRONT_DEPTH_MAX_HEIGHT' in os.environ:
    front_depth_min_height = float(os.environ.get('FRONT_DEPTH_MIN_HEIGHT'))
    front_depth_max_height = float(os.environ.get('FRONT_DEPTH_MAX_HEIGHT'))
else:
    front_depth_min_height = -0.35
    front_depth_max_height = 1.0

if 'LEFT_DEPTH_MIN_HEIGHT' and 'LEFT_DEPTH_MAX_HEIGHT' in os.environ:
    left_depth_min_height = float(os.environ.get('LEFT_DEPTH_MIN_HEIGHT'))
    left_depth_max_height = float(os.environ.get('LEFT_DEPTH_MAX_HEIGHT'))
else:
    left_depth_min_height = -0.35
    left_depth_max_height = 1.5

if 'RIGHT_DEPTH_MIN_HEIGHT' and 'RIGHT_DEPTH_MAX_HEIGHT' in os.environ:
    right_depth_min_height = float(os.environ.get('RIGHT_DEPTH_MIN_HEIGHT'))
    right_depth_max_height = float(os.environ.get('RIGHT_DEPTH_MAX_HEIGHT'))
else:
    right_depth_min_height = -0.35
    right_depth_max_height = 1.5

front_camera = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': front_depth_min_height,
        'max_height': front_depth_max_height,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.2,
        'range_max': 10.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

right_camera = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': right_depth_min_height,
        'max_height': right_depth_max_height,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.2,
        'range_max': 10.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

back_camera = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': back_depth_min_height,
        'max_height': back_depth_max_height,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.2,
        'range_max': 10.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

left_camera = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': left_depth_min_height,
        'max_height': left_depth_max_height,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.2,
        'range_max': 10.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]


node_front = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', 'camera1/depth/points'),
                ('scan', 'point_scan_front')],
    parameters=front_camera,
    name='pointcloud_to_laserscan_front'
    )

node_right = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', 'camera2/depth/points'),
                ('scan', 'point_scan_right')],
    parameters=right_camera,
    name='pointcloud_to_laserscan_right'
    )

node_back = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', 'camera3/depth/points'),
                ('scan', 'point_scan_back')],
    parameters=back_camera,
    name='pointcloud_to_laserscan_back'
    )

node_left = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in', 'camera4/depth/points'),
                ('scan', 'point_scan_left')],
    parameters=left_camera,
    name='pointcloud_to_laserscan_left'
    )




def generate_launch_description():
    
    return LaunchDescription([
        node_front,
        node_right,
        node_back,
        node_left    
    ])
