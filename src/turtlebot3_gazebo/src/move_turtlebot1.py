import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point


class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('turtlebot_mover')

        # Publisher for robot movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for Z-axis translation
        self.subscriber = self.create_subscription(
            Point,
            'aruco_marker_translation',  # Topic name where Z-axis data is published
            self.translation_callback,
            10
        )

        self.z_distance = 0.0  # Variable to store Z-axis distance
        self.get_logger().info('TurtleBot Mover Node Started')

    def translation_callback(self, msg):
        """Callback to handle data from the ArUco marker translation topic."""
        self.z_distance = msg.z  # Extract Z-axis translation
        self.get_logger().info(f'Received Z-distance: {self.z_distance:.2f} m')

        # Move the robot based on Z-distance
        self.move_based_on_distance()

    def move_based_on_distance(self):
        """Move forward if Z-distance > 1 meter, else stop."""
        msg = Twist()
        if self.z_distance > .1:  # Distance threshold to move forward
            msg.linear.x = 0.2  # Set forward velocity (in meters/second)
            self.get_logger().info('Moving forward')
        else:
            msg.linear.x = 0.0  # Stop the robot
            self.get_logger().info('Stopping')

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
