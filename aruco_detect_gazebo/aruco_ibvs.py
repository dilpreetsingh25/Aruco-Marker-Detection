import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class IBVSNode(Node):
    def __init__(self):
        super().__init__('ibvs_node')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('marker_size', 0.05)  # Marker size in meters
        
        # Topics
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        
        # Initialize components
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        
        self.get_logger().info(f'Subscribed to {camera_topic}')

        # Load ArUco dictionary and detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Camera calibration
        self.camera_matrix = self.load_camera_matrix('/home/dilpreet/ros2_ws/src/aruco_package1/aruco_pose_est_cam/aruco_marker_pose_esti.yaml')
        self.dist_coeffs = self.load_dist_coeffs('/home/dilpreet/ros2_ws/src/aruco_package1/aruco_pose_est_cam/aruco_marker_pose_esti.yaml')

        # Target parameters
        self.target_center = np.array([320, 240])  # Image center (example for 640x480 resolution)
        self.control_gain = 0.1  # Gain for velocity control

        # Configure OpenCV window
        cv2.namedWindow("Aruco Markers", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Aruco Markers", 800, 600)
                         
        # OpenCV window
        cv2.namedWindow("IBVS Markers", cv2.WINDOW_NORMAL)

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

    def image_callback(self, msg):
        # Convert image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # Estimate pose of the markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i, corner in enumerate(corners):
                # Calculate marker center
                marker_center = np.mean(corner[0], axis=0)
                
                # Draw detected markers and axes
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                # IBVS control
                error = self.target_center - marker_center
                self.get_logger().info(f"Error: {error}")

                # Generate velocity command
                twist_msg = Twist()
                twist_msg.linear.x = self.control_gain * error[1]  # Control forward/backward
                twist_msg.angular.z = -self.control_gain * error[0]  # Control rotation

                # Publish velocity command
                self.cmd_vel_pub.publish(twist_msg)

        # Display image
        cv2.imshow("IBVS Markers", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = IBVSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
