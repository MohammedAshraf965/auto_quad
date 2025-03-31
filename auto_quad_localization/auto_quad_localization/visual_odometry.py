import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import yaml

class VisualOdometry(Node):
    def __init__(self):
        super().__init__('visual_odometry')
        # Subscriber for disparity map
        self.disparity_sub = self.create_subscription(
            Image, '/disparity', self.disparity_callback, 10)
        # Tools
        self.bridge = CvBridge()
        # Load calibration data
        self.load_calibration_data()
        # Variables for tracking
        self.prev_disparity = None
        self.prev_points = None
        self.prev_R = np.eye(3)  # Initial rotation
        self.prev_t = np.zeros((3, 1))  # Initial translation

    def load_calibration_data(self):

        package_share_dir = get_package_share_directory('auto_quad_localization')
        config_dir = os.path.join(package_share_dir, 'config')

        # Load calibration files from the config directory
        left_calib_file = os.path.join(config_dir, 'left_camera_info.yaml')
        right_calib_file = os.path.join(config_dir, 'right_camera_info.yaml')

        # Load left camera calibration
        with open(left_calib_file, 'r') as f:
            left_calib = yaml.safe_load(f)
        # Load right camera calibration
        with open(right_calib_file, 'r') as f:
            right_calib = yaml.safe_load(f)

        # Extract projection matrices
        self.P1 = np.array(left_calib['projection_matrix']['data']).reshape(3, 4)
        self.P2 = np.array(right_calib['projection_matrix']['data']).reshape(3, 4)
        self.fx = self.P1[0, 0]  # Rectified focal length
        tx = self.P2[0, 3]  # Baseline in pixels
        self.baseline = -tx / self.fx  # Baseline in meters
        self.get_logger().info(f"Loaded calibration: fx={self.fx}, baseline={self.baseline} meters")

    def disparity_callback(self, msg):
        try:
            # Convert disparity map to OpenCV format
            disparity = self.bridge.imgmsg_to_cv2(msg, '32FC1')

            # Triangulate 3D points
            points_3d = self.triangulate_points(disparity)

            # If this is the first frame, store the points and return
            if self.prev_points is None:
                self.prev_points = points_3d
                self.prev_disparity = disparity
                return

            # Estimate motion between frames
            R, t = self.estimate_motion(points_3d)
            self.prev_R = R @ self.prev_R
            self.prev_t = self.prev_t + R @ t

            # Log the current pose
            self.get_logger().info(f"Current translation: {self.prev_t.flatten()}")
            self.get_logger().info(f"Current rotation: {self.prev_R}")

            # Update previous points and disparity
            self.prev_points = points_3d
            self.prev_disparity = disparity

        except Exception as e:
            self.get_logger().error(f"Error in disparity_callback: {str(e)}")

    def triangulate_points(self, disparity):
        # Create a grid of pixel coordinates
        h, w = disparity.shape
        x, y = np.meshgrid(np.arange(w), np.arange(h))
        mask = (disparity > 0) & (np.isfinite(disparity))

        # Compute 3D points
        z = (self.baseline * self.fx) / disparity[mask]  # Depth
        x_3d = (x[mask] - self.P1[0, 2]) * z / self.fx  # X
        y_3d = (y[mask] - self.P1[1, 2]) * z / self.fx  # Y
        points_3d = np.vstack((x_3d, y_3d, z)).T

        return points_3d

    def estimate_motion(self, points_3d):
        # For simplicity, assume we have 2D points in the current frame (e.g., from optical flow)
        # Here, we'll use the disparity map to find corresponding 2D points
        # In a real VO pipeline, you'd track features (e.g., using ORB) across frames
        # For now, let's use the 3D points and assume we have matches

        # Randomly sample points for RANSAC
        indices = np.random.choice(len(points_3d), size=min(100, len(points_3d)), replace=False)
        curr_points = points_3d[indices]

        # Find corresponding points in the previous frame (simplified)
        prev_points = self.prev_points[indices]

        # Estimate essential matrix
        E, mask = cv2.findEssentialMat(curr_points, prev_points, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        _, R, t, mask = cv2.recoverPose(E, curr_points, prev_points)

        return R, t

def main():
    rclpy.init()
    node = VisualOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()