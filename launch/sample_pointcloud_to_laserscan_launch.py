from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

front_camera = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': -0.927,
        'max_height': 0.5,
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
        'min_height': -0.40,
        'max_height': 1.0,
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
        'min_height': -0.35,
        'max_height': 1.0,
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
        'min_height': -0.35,
        'max_height': 1.0,
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
