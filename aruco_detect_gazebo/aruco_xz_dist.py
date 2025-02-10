import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('camera_topic', '/camera/image_raw')  # Update topic here
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Subscriber for camera feed
        self.subscriber = self.create_subscription(
            Image,
            camera_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {camera_topic}')

        # Publisher for Z-axis translation
        self.publisher_ = self.create_publisher(Point, 'aruco_marker_translation', 10)
        # self.timer_ = self.create_timer(1,self.listener_callback)
        self.get_logger().info('Publisher created for ArUco marker translation')

        # ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Camera calibration parameters (example values, replace with yours)
        # self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        # self.dist_coeffs = np.zeros((5,), dtype=np.float32)
        self.camera_matrix = self.load_camera_matrix('/home/dilpreet/ros2_ws/src/aruco_package1/aruco_pose_est_cam/aruco_marker_pose_esti.yaml')
        self.dist_coeffs = self.load_dist_coeffs('/home/dilpreet/ros2_ws/src/aruco_package1/aruco_pose_est_cam/aruco_marker_pose_esti.yaml')

        # Configure OpenCV window
        cv2.namedWindow("Aruco Markers", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Aruco Markers", 800, 600)  # Set window size to fit screen

    def load_camera_matrix(self, filepath):
        cv_file = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
        camera_matrix = cv_file.getNode('K').mat()
        cv_file.release()
        return camera_matrix

    def load_dist_coeffs(self, filepath):
        cv_file = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
        dist_coeffs = cv_file.getNode('D').mat()
        cv_file.release()
        return dist_coeffs
    
    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
    
        if ids is not None:
            # Assume all markers have the same size
            marker_length = 0.05  # Marker size in meters (update based on your setup)
    
            # Estimate pose of the marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, self.camera_matrix, self.dist_coeffs)
    
            for i in range(len(ids)):
                # Draw the detected marker
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
    
                # Extract Z-axis translation (tvecs is in meters)
                z_translation = tvecs[i][0][2]  # Z-axis value of the translation vector
                self.get_logger().info(f'Marker ID {ids[i][0]}: Z-axis translation = {z_translation:.2f} m')
    
                x_translation = tvecs[i][0][0]  # X-axis value of the translation vector
                self.get_logger().info(f'Marker ID {ids[i][0]}: X-axis translation = {x_translation:.2f} m')
    
                # Publish the translation as a Point message
                point_msg = Point()
                point_msg.x = tvecs[i][0][0]  # X-axis translation
                point_msg.y = tvecs[i][0][1]  # Y-axis translation
                point_msg.z = z_translation   # Z-axis translation
                self.publisher_.publish(point_msg)
        else:
            # Publish zero message when no marker is detected
            self.get_logger().info('No markers detected. Publishing zeros.')
            point_msg = Point()
            point_msg.x = 0.0
            point_msg.y = 0.0
            point_msg.z = 0.0
            self.publisher_.publish(point_msg)
    
        # Display the image with markers
        cv2.imshow('Aruco Markers', cv_image)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
