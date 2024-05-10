from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

leaf_size_x = 0.01
leaf_size_y = 0.1
leaf_size_z = 0.1

parameters_back = [{
        'target_frame': 'camera3_color_frame',
        'transform_tolerance': 0.01,
        'min_height': -0.1,
        'max_height': 1.8,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 5.0,
        'use_inf': True,
        'inf_epsilon': 1.0,
        'leaf_size_x': leaf_size_x,
        'leaf_size_y': leaf_size_y,
        'leaf_size_z': leaf_size_z,
    }]

parameters_front_up = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': -0.1,
        'max_height': 1.8,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 5.0,
        'use_inf': True,
        'inf_epsilon': 1.0,
        'leaf_size_x': leaf_size_x,
        'leaf_size_y': leaf_size_y,
        'leaf_size_z': leaf_size_z,
    }]

parameters_front_down = [{
        'target_frame': 'base_link',
        'transform_tolerance': 0.01,
        'min_height': -0.5,
        'max_height': 1.8,
        'angle_min': -0.614,
        'angle_max': 0.614,
        'angle_increment': 0.00766,
        'scan_time': 0.3333,
        'range_min': 0.3,
        'range_max': 5.0,
        'use_inf': True,
        'inf_epsilon': 1.0,
        'leaf_size_x': leaf_size_x,
        'leaf_size_y': leaf_size_y,
        'leaf_size_z': leaf_size_z,
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
    # remappings=[('cloud_in', '/points_cloud_person'),
                # ('scan', 'point_scan_front_up')], 
    remappings=[('cloud_in', '/camera1/depth/points'),
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
        # node_front_down,
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
