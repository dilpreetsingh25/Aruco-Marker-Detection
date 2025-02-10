import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# import aruco_marker_pose_estimator.py 
class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('camera_topic', '/camera_sensor/image_raw')  # Update topic here
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(
            Image,
            camera_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f'Subscribed to {camera_topic}')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
# cv2.aruco.Dete
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
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
