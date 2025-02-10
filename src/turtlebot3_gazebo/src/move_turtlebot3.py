import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('turtlebot_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1, self.move_forward)  # Publish at 10 Hz
        self.get_logger().info('TurtleBot Mover Node Started')
        
    def move_forward(self):
        msg = Twist()
        msg.linear.z = .0  # Forward velocity (m/s)
        msg.angular.z = -0.0  # No rotation
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: Linear x: %.2f, Angular z: %.2f' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
