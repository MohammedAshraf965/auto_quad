#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.left_pub = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.right_pub = self.create_publisher(CameraInfo, '/right/camera_info', 10)
        self.timer = self.create_timer(0.1, self.publish_camera_info)

        # Get the package share directory
        package_share_dir = get_package_share_directory('auto_quad_localization')
        config_dir = os.path.join(package_share_dir, 'config')

        # Load calibration files from the config directory
        left_file = os.path.join(config_dir, 'left_camera_info.yaml')
        right_file = os.path.join(config_dir, 'right_camera_info.yaml')

        try:
            with open(left_file, 'r') as f:
                left_data = yaml.safe_load(f)
            with open(right_file, 'r') as f:
                right_data = yaml.safe_load(f)
        except FileNotFoundError as e:
            self.get_logger().error(f"Could not find file: {e}")
            raise

        # Populate left camera info
        self.left_info = CameraInfo()
        self.left_info.header.frame_id = 'left_camera'
        self.left_info.width = left_data['image_width']
        self.left_info.height = left_data['image_height']
        self.left_info.distortion_model = left_data['distortion_model']
        self.left_info.d = left_data['distortion_coefficients']['data']
        self.left_info.k = left_data['camera_matrix']['data']
        if 'rectification_matrix' in left_data:
            self.left_info.r = left_data['rectification_matrix']['data']
        if 'projection_matrix' in left_data:
            self.left_info.p = left_data['projection_matrix']['data']

        # Populate right camera info
        self.right_info = CameraInfo()
        self.right_info.header.frame_id = 'right_camera'
        self.right_info.width = right_data['image_width']
        self.right_info.height = right_data['image_height']
        self.right_info.distortion_model = right_data['distortion_model']
        self.right_info.d = right_data['distortion_coefficients']['data']
        self.right_info.k = right_data['camera_matrix']['data']
        if 'rectification_matrix' in right_data:
            self.right_info.r = right_data['rectification_matrix']['data']
        if 'projection_matrix' in right_data:
            self.right_info.p = right_data['projection_matrix']['data']

    def publish_camera_info(self):
        self.left_info.header.stamp = self.get_clock().now().to_msg()
        self.right_info.header.stamp = self.get_clock().now().to_msg()
        self.left_pub.publish(self.left_info)
        self.right_pub.publish(self.right_info)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()