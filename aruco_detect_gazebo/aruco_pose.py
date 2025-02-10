import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_pose')
        # for robot_bringup
        self.declare_parameter('camera_topic', '/camera_sensor/image_raw')                
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.subscriber = self.create_subscription( 
            Image, camera_topic, self.listener_callback, 10
        )
        self.get_logger().info(f'Subscribed to {camera_topic}')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Camera matrix and distortion coefficients
        self.camera_matrix = self.load_camera_matrix('/home/dilpreet/ros2_ws/src/aruco_package1/aruco_pose_est_cam/aruco_marker_pose_esti.yaml')
        self.dist_coeffs = self.load_dist_coeffs('/home/dilpreet/ros2_ws/src/aruco_package1/aruco_pose_est_cam/aruco_marker_pose_esti.yaml')

        # YAML file to store the pose data
        self.yaml_file = '/home/dilpreet/ros2_ws/src/aruco_package1/aruco_detect_gazebo/aruco_pose.yaml'

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

    def save_pose_to_yaml(self, rvecs, tvecs, ids):
        cv_file = cv2.FileStorage(self.yaml_file, cv2.FILE_STORAGE_WRITE)
        for i in range(len(ids)):
            cv_file.write(f'rvec_{ids[i][0]}', rvecs[i])
            cv_file.write(f'tvec_{ids[i][0]}', tvecs[i])
        cv_file.release()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is not None:
            # Estimate the pose of the markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.05, self.camera_matrix, self.dist_coeffs
            )
            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(cv_image, corners)
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                self.get_logger().info(f"Marker ID: {ids[i][0]}")
                self.get_logger().info(f"Translation Vector: {tvecs[i].flatten()}")
                self.get_logger().info(f"Rotation Vector: {rvecs[i].flatten()}")


            # Save pose data to YAML
            self.save_pose_to_yaml(rvecs, tvecs, ids)

            for rvec, tvec in zip(rvecs, tvecs):
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

        cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

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
