import rclpy
import math
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import numpy as np


class PotentialFieldController(Node):
    '''
    Node that will generate potential fields around objects, so in autonoumous driving the car can avoid obstacles
    '''
    def __init__(self):
        super().__init__('potentialField_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar, 10)
        self.subscription
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.position_callback, 10)
        self.odom_subscription
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', qos_profile_sensor_data)  

        #Robot Constraints
        self.maxVelocity = 3.0
        self.maxSteeringAngle = 0.34
        self.minSteeringAngle = -0.34

        #Closest object
        self.closestObject = 0

        #Scaling Factors
        self.scalingFactorAttract = 0.5
        self.scalingFactorRepulse = 0.5  
        self.distanceInfluence = 0.5   

        #Robots (x,y) position
        self.position = (0,0)

        #Goal position written as (x,y)
        self.goalPosition = (5,5)

        #Attract and Repulse Vector 
        self.attractForce = (0,0)
        self.repulseForce = (0,0)
        self.netForce = (0,0)

    def attract_force(self):
        #TODO
        force_x = 0.5 * self.scalingFactorAttract * np.abs(self.position[0] - self.goalPosition[0])
        force_y = 0.5 * self.scalingFactorAttract * np.abs(self.position[1] - self.goalPosition[1])
        self.attractForce = (force_x, force_y)
        

    def repulse_force(self):
        #TODO
        # x = np.abs(self.position[0] - self.goalPosition[0])
        # y = np.abs(self.position[1] - self.goalPosition[1])


        #net force that needs to be split into x and y components
        net_force = 0.5 * self.scalingFactorRepulse * ((1/self.closestObject) - self.distanceInfluence)**2

        #angle to split vector into components
        angle = math.atan2(self.position[1], self.position[0])

        x = net_force * math.cos(angle)
        y = net_force * math.sin(angle)


        #Checks if the object is close enough that the robot needs to avoid
        if x >= self.distanceInfluence:
            x = 0.0
        if y >= self.distanceInfluence:
            y = 0.0

        #Not Right
        # if x <= self.distanceInfluence:
        #     x = 0.5 * self.scalingFactorRepulse * ((1/self.closestObject) - self.distanceInfluence)**2
        # else:
        #     x = 0
        
        # if y <= self.distanceInfluence:
        #     y = 0.5 * self.scalingFactorRepulse * ((1/self.closestObject) - self.distanceInfluence)**2
        # else:
        #     y = 0

        self.repulseForce = (x,y)
        
    
    def net_force(self):
        #TODO
        self.netForce = np.add(np.array(self.attractForce), np.array(self.repulseForce))
        self.drive()
    
    def position_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def lidar(self, msg):
        self.closestObject = min(msg.ranges)
    
    def drive(self):
        velocity = np.linalg.norm(self.netForce)
        if velocity > self.maxVelocity:
            velocity = self.maxVelocity

        steering_angle = math.atan2(self.netForce[1], self.netForce[0])

        if steering_angle < self.minSteeringAngle:
            steering_angle = self.minSteeringAngle
        
        if steering_angle > self.maxSteeringAngle:
            steering_angle = self.maxSteeringAngle
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = steering_angle

        self.ackermann_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    potential_field_controller = PotentialFieldController()
    rclpy.spin(potential_field_controller)
    potential_field_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


