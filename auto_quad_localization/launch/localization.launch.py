from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='stereo_camera',
            parameters=[
                {'video_device': '/dev/video4'},
                {'image_size': [1280, 480]}
            ]
        ),
        Node(
            package='auto_quad_localization',
            executable='stereo_feedback',
            name='stereo_feedback',
            output='screen'
        ),
       Node(
            package='auto_quad_localization',
            executable='camera_calibration_service',
            name='camera_calibration_service',
            output='screen'
        ),
        Node(
            package='auto_quad_localization',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen'
        ),
        Node(
            package='auto_quad_localization',
            executable='feature_matcher',
            name='feature_matcher',
            output='screen'
        )
        # Node(
        #     package='auto_quad_localization',
        #     executable='visual_odometry',
        #     name='visual_odometry',
        #     output='screen'
        # )
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_left_camera',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'left_camera']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_right_camera',
        #     arguments=['0.1', '0', '0', '0', '0', '0', 'base_link', 'right_camera']  # Adjust x for baseline
        # )
        # Node(
        #     package='rtabmap_slam',
        #     executable='rtabmap',
        #     name='rtabmap',
        #     parameters=[{
        #         'frame_id': 'base_link',             # Frame of your camera
        #         'subscribe_depth': False,            # No depth for stereo
        #         'subscribe_rgbd': False,             # No RGB-D
        #         'subscribe_stereo': True,            # Use stereo input
        #         'left_image_topic': '/left/image_raw',
        #         'right_image_topic': '/right/image_raw',
        #         # 'left_camera_info_topic': '/left/camera_info',
        #         # 'right_camera_info_topic': '/right/camera_info',
        #         'approx_sync': True,                 # Approximate time sync
        #         'queue_size': 50,
        #         'publish_tf': True,                  # Publish transforms
        #         'odom_frame_id': 'odom',             # Odometry frame
        #         'map_frame_id': 'map',               # Map frame
        #         'visual_odometry': True,             # Enable visual odometry
        #         'icp_odometry': False,               # Disable ICP odometry
        #         'use_sim_time': False,
        #         'Vis/MaxFeatures': 2000,  # Increase feature detection
        #         'Vis/MinInliers': 10,     # Minimum inliers for odometry
        #         'Vis/EstimationType': 1,  # 1 = 3D->2D (PnP), more robust for stereo
        #         'Vis/FeatureType': 6,     # 6 = SURF, 7 = SIFT, 8 = ORB (choose based on performance)
        #         'Stereo/WinWidth': 15,    # Stereo matching window size
        #         'Stereo/WinHeight': 15,
        #         'Stereo/MaxDisparity': 128,  # Maximum disparity for stereo matching
        #     }],
        #     remappings=[
        #         ('left/image_rect', '/left/image_raw'),
        #         ('right/image_rect', '/right/image_raw'),
        #         ('left/camera_info', '/left/camera_info'),
        #         ('right/camera_info', '/right/camera_info'),
        #     ],
        #     output='screen'
        # )
        # Uncomment the following to launch RViz2 automatically
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'
        # )
])