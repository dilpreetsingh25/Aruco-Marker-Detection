#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

class TurtleBotMover : public rclcpp::Node {
public:
    TurtleBotMover()
    : Node("turtlebot_mover"), z_distance_(0.0) {
        // Publisher for robot movement
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for Z-axis translation
        subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "aruco_marker_translation",  // Topic name where Z-axis data is published
            10,
            std::bind(&TurtleBotMover::translation_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "TurtleBot Mover Node Started");
    }

private:
    void translation_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // Callback to handle data from the ArUco marker translation topic
        z_distance_ = msg->z;  // Extract Z-axis translation
        RCLCPP_INFO(this->get_logger(), "Received Z-distance: %.2f m", z_distance_);

        // Check if Z-distance > 0.1 and move forward
        move_based_on_distance();
    }

    void move_based_on_distance() {
        // Create Twist message
        auto msg = geometry_msgs::msg::Twist();

        if (z_distance_ > 0.1) {
            msg.linear.x = 0.2;  // Set forward velocity (in meters/second)
            RCLCPP_INFO(this->get_logger(), "Moving forward");
        } else {
            msg.linear.x = 0.0;  // Stop the robot
            RCLCPP_INFO(this->get_logger(), "Stopping");
        }

        // Publish the movement command
        publisher_->publish(msg);
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;
    double z_distance_;  // Variable to store Z-axis distance
};

int main(int argc, char *argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the TurtleBotMover node
    auto node = std::make_shared<TurtleBotMover>();

    // Keep the node running
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
