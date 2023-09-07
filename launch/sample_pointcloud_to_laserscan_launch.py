from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

parameters_back = [{
        'target_frame': 'camera3_color_frame',
        'transform_tolerance': 0.01,
        'min_height': -0.1,
        'max_height': 1.8,
        'angle_min': -0.68941,
        'angle_max': 0.68941,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 3.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

parameters_front_up = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': -0.1,
        'max_height': 1.8,
        'angle_min': -0.68941,
        'angle_max': 0.68941,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 3.0,
        'use_inf': True,
        'inf_epsilon': 1.0
    }]

parameters_front_down = [{
        'target_frame': 'camera2_color_frame',
        'transform_tolerance': 0.01,
        'min_height': -0.1,
        'max_height': 1.8,
        'angle_min': -0.68941,
        'angle_max': 0.68941,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 3.0,
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
        # DeclareLaunchArgument(
        #     name='scanner', default_value='scanner',
        #     description='Namespace for sample topics'
        # ),
        # Node(
        #     package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
        #     remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
        #     parameters=[{'cloud_frame_id': 'cloud', 'cloud_extent': 2.0, 'cloud_size': 500}],
        #     name='cloud_publisher'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'map', '--child-frame-id', 'cloud'
        #     ]
        # ),        
    ])
