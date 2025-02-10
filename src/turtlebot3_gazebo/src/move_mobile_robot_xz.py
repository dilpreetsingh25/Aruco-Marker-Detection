import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('turtlebot_mover')

        # Publisher for robot movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for translation data (X and Z axis)
        self.subscriber = self.create_subscription(
            Point,
            'aruco_marker_translation',  # Topic where X and Z data are published
            self.translation_callback,
            10
        )

        self.z_distance = 0.0  # Variable to store Z-axis distance
        self.x_distance = 0.0  # Variable to store X-axis distance
        self.get_logger().info('TurtleBot Mover Node Started')

    # def translation_callback(self, msg):
    #     """Callback to handle data from the ArUco marker translation topic."""
    #     self.z_distance = msg.z  # Extract Z-axis translation
    #     self.x_distance = msg.x  # Extract X-axis translation
    #     self.get_logger().info(f'Received Z-distance: {self.z_distance:.2f} m, X-distance: {self.x_distance:.2f} m')
        
    #     # Move robot based on translation distances
    #     self.move_based_on_distance()

    def translation_callback(self, msg):
        """Callback to handle data from the ArUco marker translation topic."""
        # Check if the ArUco marker is detected
        if msg.z != 0.0 and msg.x != 0.0:  # Assuming a marker is detected when Z and X distances are non-zero
            self.z_distance = msg.z  # Extract Z-axis translation
            self.x_distance = msg.x  # Extract X-axis translation
            self.get_logger().info(f'Detected ArUco Marker - Z-distance: {self.z_distance:.2f} m, X-distance: {self.x_distance:.2f} m')

            # Move the robot based on translation distances
            self.move_based_on_distance()
        else:
            # Rotate the TurtleBot3 to search for the marker
            msg = Twist()
            msg.angular.z = 0.1  # Rotate at 0.5 radians/second
            self.publisher.publish(msg)
            self.get_logger().info('ArUco Marker not detected - Rotating to search')


    def move_based_on_distance(self):
        """Move forward if Z-distance > 0.1 meter, else stop. You can also add X-axis movement logic."""
        msg = Twist()

        # Check if Z-distance > 0.1 meter and move forward
        if self.z_distance > 0.15:
            msg.linear.x = 0.3  # Set forward velocity (in meters/second)
            self.get_logger().info('Moving forward based on Z-distance')
        else:
            msg.linear.x = 0.0  # Stop the robot
            self.get_logger().info('Stopping based on Z-distance')

        # If X-distance > 0.2 meters, move the robot in the Y-direction (optional)
        if self.z_distance <0.0:
            if self.x_distance < 0.5:
                msg.angular.z = 0.1  # Move in the Y direction (if required)
                self.get_logger().info('Moving based on X-distance')
            else:
                msg.linear.y = 0.0  # No movement in the Y direction

        # Publish the movement command
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
