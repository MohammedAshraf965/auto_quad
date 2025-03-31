#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.srv import SetCameraInfo

class CameraInfoService(Node):
    def __init__(self):
        super().__init__('camera_info_service')
        self.left_srv = self.create_service(SetCameraInfo, '/left_camera/set_camera_info', self.set_camera_info_callback)
        self.right_srv = self.create_service(SetCameraInfo, '/right_camera/set_camera_info', self.set_camera_info_callback)

    def set_camera_info_callback(self, request, response):
        self.get_logger().info('Received camera info')
        response.success = True  # Pretend it saved (you can add file saving here)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()