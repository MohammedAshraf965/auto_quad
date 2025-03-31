import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import message_filters
import os
from ament_index_python.packages import get_package_share_directory
import yaml

class StereoFeatureMatcher(Node):
    def __init__(self):
        super().__init__('stereo_feature_matcher')

        # Subscribers for left and right images
        self.left_sub = message_filters.Subscriber(self, Image, '/left/image_gray')
        self.right_sub = message_filters.Subscriber(self, Image, '/right/image_gray')
        
        # Publisher for the matched features
        self.feature_matcher_pub = self.create_publisher(Image, '/features', 10)
        self.disparity_pub = self.create_publisher(Image, '/disparity', 10)
        
        # Synchronize the left and right images
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)
        
        # Tools
        self.bridge = CvBridge()

        self.orb = cv2.ORB_create()
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)  # Brute-Force Matcher
        # self.stereo = cv2.StereoBM_create(numDisparities=16*6, blockSize=31)

        window_size = 11
        self.stereo = cv2.StereoSGBM_create(
                                    minDisparity=0,
                                    numDisparities=96,
                                    blockSize=window_size,
                                    P1=8 * 3 * window_size * window_size,  # Controls smoothness (3 channels for grayscale)
                                    P2=32 * 3 * window_size * window_size,
                                    disp12MaxDiff=1,
                                    uniquenessRatio=5,
                                    speckleWindowSize=200,
                                    speckleRange=2,
                                    preFilterCap=31,
                                    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
                                )
        
        # WLS filter for post-processing
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
        self.wls_filter.setLambda(8000)
        self.wls_filter.setSigmaColor(1.5)

        # Storage for images and features
        self.left_img = None
        self.right_img = None
        self.left_kp = None
        self.right_kp = None
        self.left_des = None
        self.right_des = None

        self.load_calibration_data()

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

        self.left_camera_matrix = np.array(left_calib['camera_matrix']['data']).reshape(3, 3)
        self.left_dist_coeffs = np.array(left_calib['distortion_coefficients']['data'])
        self.right_camera_matrix = np.array(right_calib['camera_matrix']['data']).reshape(3, 3)
        self.right_dist_coeffs = np.array(right_calib['distortion_coefficients']['data'])

        # Extract rectification and projection matrices
        self.R1 = np.array(left_calib['rectification_matrix']['data']).reshape(3, 3)
        self.R2 = np.array(right_calib['rectification_matrix']['data']).reshape(3, 3)
        self.P1 = np.array(left_calib['projection_matrix']['data']).reshape(3, 4)
        self.P2 = np.array(right_calib['projection_matrix']['data']).reshape(3, 4)

        # Image dimensions
        self.image_size = (left_calib['image_width'], left_calib['image_height'])

        # Compute rectification maps
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            self.left_camera_matrix, self.left_dist_coeffs, self.R1, self.P1,
            self.image_size, cv2.CV_16SC2)
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            self.right_camera_matrix, self.right_dist_coeffs, self.R2, self.P2,
            self.image_size, cv2.CV_16SC2)

        # Extract rectified focal length (fx') and baseline
        self.fx = self.P1[0, 0]
        tx = self.P2[0, 3]
        self.baseline = -tx / self.fx
        self.get_logger().info(f"Loaded calibration: fx={self.fx}, baseline={self.baseline} meters")

    def synced_callback(self, left_msg, right_msg):
        try:
            # Convert ROS Image messages to OpenCV format (BGR)
            self.left_img = self.bridge.imgmsg_to_cv2(left_msg, 'mono8')
            self.right_img = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')
            
            # Detect keypoints and compute descriptors
            self.left_kp, self.left_des = self.orb.detectAndCompute(self.left_img, None)
            self.right_kp, self.right_des = self.orb.detectAndCompute(self.right_img, None)

            # Check if features were detected
            if self.left_des is None or self.right_des is None:
                self.get_logger().warn("No features detected in one or both images")
                return

            # Match descriptors
            matches = self.bf.match(self.left_des, self.right_des)
            
            # Sort matches by distance (lower is better)
            matches = sorted(matches, key=lambda x: x.distance)

            # Visualize top 10 matches
            img_matches = cv2.drawMatches(
                self.left_img, self.left_kp, self.right_img, self.right_kp,
                matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            
            # Convert the image to ROS message and publish
            features_msg = self.bridge.cv2_to_imgmsg(img_matches)
            self.feature_matcher_pub.publish(features_msg)
            
            # Display the matches
            # cv2.imshow('Stereo Matches', img_matches)
            # cv2.waitKey(1)
            # self.get_logger().info(f"Found {len(matches)} matches")


            ######################################## Disparity Map Calculation ########################################

            # Rectify the images
            left_rect = cv2.remap(self.left_img, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            right_rect = cv2.remap(self.right_img, self.right_map1, self.right_map2, cv2.INTER_LINEAR)

            # Compute disparity map
            disparity = self.stereo.compute(left_rect, right_rect)

            # cv2.imshow('Disparity', disparity)
            # cv2.waitKey(1)

            # Normalize disparity for visualization (optional)
            disparity_norm = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            cv2.imshow('Disparity Norm', disparity_norm)
            cv2.waitKey(1)

            # Compute depth
            disparity_float = disparity.astype(np.float32) / 16.0  # StereoBM returns disparity * 16

            cv2.imshow('Disparity Map', disparity_float)
            cv2.waitKey(1)

            # Convert disparity to ROS Image message 
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity_float, encoding='32FC1')
            disparity_msg.header = left_msg.header
            self.disparity_pub.publish(disparity_msg)
            self.get_logger().info("Published disparity map")

        except Exception as e:
            self.get_logger().error(f"Error in synced_callback: {str(e)}")

def main():
    rclpy.init()
    node = StereoFeatureMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()