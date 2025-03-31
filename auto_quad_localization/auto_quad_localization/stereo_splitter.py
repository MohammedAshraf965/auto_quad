import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class StereoSplitter(Node):
    def __init__(self):
        super().__init__('stereo_splitter')
        # Subscribe to the compressed stereo image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10)
        # Publishers for left and right images (color and grayscale)
        self.left_color_pub = self.create_publisher(Image, '/left/image_raw', 10)
        self.right_color_pub = self.create_publisher(Image, '/right/image_raw', 10)
        # Initialize CvBridge for converting between OpenCV and ROS image formats
        self.bridge = CvBridge()
        self.get_logger().info("StereoSplitter node started, subscribing to 'image_raw/compressed'")

    def image_callback(self, msg):
        try:
            # Decode the compressed image
            buf = np.frombuffer(msg.data, np.uint8)
            combined_image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if combined_image is None:
                self.get_logger().error("Failed to decode compressed image")
                return

            # Split the image into left and right halves
            height, width, _ = combined_image.shape
            left_image = combined_image[:, :width//2, :]
            right_image = combined_image[:, width//2:, :]

            # Create ROS image messages
            left_color_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='bgr8')
            right_color_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='bgr8')

            # Set headers with timestamps and frame IDs
            left_color_msg.header = msg.header
            right_color_msg.header = msg.header
            # Assign distinct frame IDs for left and right cameras
            left_color_msg.header.frame_id = 'left_camera'
            right_color_msg.header.frame_id = 'right_camera'

            # Publish all images
            self.left_color_pub.publish(left_color_msg)
            self.right_color_pub.publish(right_color_msg)
            self.get_logger().info("Published left and right color and gray images")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main():
    rclpy.init()
    node = StereoSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()