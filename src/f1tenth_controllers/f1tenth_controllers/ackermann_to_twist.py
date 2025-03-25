#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data


class AckermannToTwist(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')
        
        # Declare and get parameters
        self.declare_parameter('wheelbase', 0.33)  # Default value for F1Tenth
        self.wheelbase = self.get_parameter('wheelbase').value
        
        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.ackermann_callback, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info(f"Initialized with wheelbase: {self.wheelbase} m")
        self.get_logger().info("Subscribing to /drive and publishing to /cmd_vel")

    def ackermann_callback(self, msg):
        twist = Twist()
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        
        # Calculate angular velocity using Ackermann kinematics
        if abs(speed) > 0.001:  # Avoid division by zero
            angular_z = (speed * math.tan(steering_angle)) / self.wheelbase
        else:
            angular_z = 0.0

        twist.linear = Vector3(x=speed, y=0.0, z=0.0)
        twist.angular = Vector3(x=0.0, y=0.0, z=angular_z)
        
        self.pub.publish(twist)
        self.get_logger().info(f"Converted - Speed: {speed:.2f} m/s, Steering: {math.degrees(steering_angle):.1f}°, Angular Z: {angular_z:.2f} rad/s")

def main():
    rclpy.init()
    node = AckermannToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()