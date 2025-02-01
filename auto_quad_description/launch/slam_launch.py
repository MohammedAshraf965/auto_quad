from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),
        # Static Transform Publisher (base_link to lidar_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_lidar_link',
            arguments=['0', '0', '0.3', '0', '0', '0', 'lidar_link', 'laser'],
        ),
        # Static Transform Publisher (base_link to lidar_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
        ),
        # Node(
        #     package='sllidar_ros2',
        #     executable='sllidar_node',
        #     name='lidar_node',
        #     parameters=[{
        #         'frame_id': 'lidar_link',  # Set the frame_id here
        #     }],
        # ),
        # SLAM Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'resolution': 0.05,
                'max_laser_range': 16.0,
            }],
            remappings=[
                ('/scan', '/scan'),
            ],
        ),
    ])