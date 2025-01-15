#ifndef TWIST_TO_ACKERMANN_HPP_
#define TWIST_TO_ACKERMANN_HPP_
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

/**
 * Issue with nav2 based controller is that they will given messages in TwistStamped format, but the robot
 * accepts commands in AckermannDriveStamped format. So this node will take the message and transform the 
 * original msg so that the robot can use it. 
 * Steps:
 * 1. Subscribe to where the controller posts msg "/cmd_vel"
 * 2. Take msg and Transform
 * 3. Publish to /drive topic
 */

class TwistToAckermann : public rclcpp::Node {
    public:
        TwistToAckermann() : Node("twistToAckermann") {

        }
    
    private:
        void cmd_vel_callback(const geometry_msgs::msg::TwistStamped msg);
        ackermann_msgs::msg::AckermannDriveStamped transform(geometry_msgs::msg::TwistStamped msg);
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
        const double f1tenth_wheelbase = 0.33;
};

#endif
