#include "f1tenth_planners/twistToAckermann.hpp"
using std::placeholders::_1;

TwistToAckermann::TwistToAckermann() : Node("twistToAckermann") {
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 20);
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel", 20, std::bind(&TwistToAckermann::cmd_vel_callback, this, _1));
}

ackermann_msgs::msg::AckermannDriveStamped TwistToAckermann::transform(geometry_msgs::msg::TwistStamped msg) {
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;

    // Copy timestamp and frame ID from the input TwistStamped
    ackermann_msg.header.stamp = msg.header.stamp;
    ackermann_msg.header.frame_id = msg.header.frame_id;

    // Extract linear velocity (m/s) and angular velocity (rad/s)
    double linear_velocity = msg.twist.linear.x;
    double angular_velocity = msg.twist.angular.z;

    // Calculate the steering angle (radians)
    if (std::abs(angular_velocity) > 1e-6) {
        ackermann_msg.drive.steering_angle = std::atan(f1tenth_wheelbase * angular_velocity / linear_velocity);
    } else {
        ackermann_msg.drive.steering_angle = 0.0; // No turning required
    }

    // Set speed (linear velocity in m/s)
    ackermann_msg.drive.speed = linear_velocity;

    return ackermann_msg;
}

void TwistToAckermann::cmd_vel_callback(const geometry_msgs::msg::TwistStamped msg) {
    ackermann_msgs::msg::AckermannDriveStamped ackermann_msg = transform(msg);
    this->publisher_->publish(ackermann_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToAckermann>());
    rclcpp::shutdown();
    return 0;
}