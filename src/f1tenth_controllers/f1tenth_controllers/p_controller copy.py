#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.duration import Duration
import time
import numpy as np
import std_msgs
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math
import tf_transformations
from collections import deque
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

import scipy
from scipy.spatial.transform import Rotation as R


#PARAMETERS
k = 0.2  # look forward gain
Lfc = 1.5  # [m] look-ahead distance
Kp = 1.0  # speed proportional gains
WB = 0.32  # [m] wheel base of vehicle
throttle_percentage = 0.3 # limit on how fast the car will travel
lf = 0.17
lr = 0.15
min_lookahead = 0.5
max_lookahead = 3.0
lookahead_ratio = 1.5

class PurePursuitController(Node):
    def __init__(self):
        # member variables
        super().__init__("pure_pursuit_node")
        self.map_path = "/home/nvidia/f1tenth_ws/src/pure_pursuit/racelines/shepherd_lab_raceline_v1.csv"
        data = np.loadtxt(self.map_path, delimiter = ",")

        self.cx = data[:, 0] # 1st column of data -> x-position of the waypoints
        self.cy = data[:, 1] # 2nd column of data -> y-position of the waypoints
        self.cv = data[:, 2] # 3rd column of data -> velocity of the waypoints
        
        self.initialize = 0
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        
        self.current_velocity = 0.0
        self.current_servo_position = 0.0
        

        # ROS subscriptions and publisher
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.odom_subscription

        self.servo_subscription = self.create_subscription(
            Float64, "/sensors/servo_position_command", self.servo_callback, 10
        )
        self.servo_subscription
        
        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, "/drive", qos_profile_sensor_data
        )

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.drive)


        self.lookahead_marker_pub = self.create_publisher(Marker, "/lookahead_marker", 5)
        self.lookahead_marker_timer = self.create_timer(0.1, self.lookahead_publish_waypoint)

        self.curr_marker_pub = self.create_publisher(Marker, "/curr_marker", 5)
        self.currmarker_timer = self.create_timer(0.1, self.curr_publish_waypoint)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform_deque = deque(maxlen=10)



        # initialization
        self.current_index = None
        self.target_index = 0
        self.i = 0

    def odom_callback(self, msg):  # update x and y position
        self.current_velocity = msg.twist.twist.linear.x # The velocity the car is going
    
    def servo_callback(self, msg):
        self.current_servo_position = msg.data - 0.51 # 0.51 is the 0
    	
    def proportional_control(self, K, target, current):
        cntrl = K * (target - current)
        return cntrl

    def lookahead_publish_waypoint(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead_waypoint"
        # self.get_logger().info(f'Target Waypoint id: {self.target_index}')
        # marker.id = int(str(self.target_index))
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.cx[self.target_index]
        marker.pose.position.y = self.cy[self.target_index]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.a = 1.0
        marker.color.r = 1.0

        self.lookahead_marker_pub.publish(marker)

    def curr_publish_waypoint(self):
        # self.get_logger().info(f'curr waypoint x: {waypoint.x}, wp_y: {waypoint.y}, wp index: {waypoint.index}')
        index = 0
        if self.current_index == None:
            index = 1
        else:
             index = self.current_index
        # self.get_logger().info(f'Current Waypoint id: {self.current_index}')
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "curr_waypoint"
        # marker.id = int(str(self.current_index))
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.cx[index]
        marker.pose.position.y = self.cy[index]

        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.a = 1.0
        marker.color.b = 0.0

        self.curr_marker_pub.publish(marker)

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
    
    def get_transform_matrix(self, translation, rotation):
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 3] = [translation.x, translation.y, translation.z]
        rotation_temp = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        rotation_matrix = rotation_temp.as_matrix()
        transform_matrix[0:3, 0:3] = rotation_matrix
        return transform_matrix
    
    def search_target_index(self):

        # To speed up nearest point search, doing it at only first time.
        if self.current_index is None:
            # search nearest point index
            dx = [self.rear_x - icx for icx in self.cx]
            dy = [self.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.current_index = ind
        else:
            ind = self.current_index
            while True:
                distance_this_index = self.calc_distance(self.cx[ind], self.cy[ind])
                distance_next_index = self.calc_distance(self.cx[(ind + 1) % len(self.cx)], self.cy[(ind + 1) % len(self.cy)])
                if distance_this_index < distance_next_index:
                    break
                ind = (ind + 1) % len(self.cx)  # Ensure wrap-around in a circular path
            self.current_index = ind

        Lf = min(max(min_lookahead, max_lookahead * self.current_velocity / lookahead_ratio), max_lookahead)
        if self.i % 20 == 0:
            self.get_logger().info(f'Lookahead Distance: {Lf}, Current Velocity: {self.current_velocity}')
            dist_to_next = self.calc_distance(self.cx[ind], self.cy[ind])
            self.get_logger().info(f'Distance to next waypoint: {dist_to_next}')
        # search look ahead target point index
        while Lf > self.calc_distance(self.cx[ind], self.cy[ind]):
            ind = (ind + 1) % len(self.cx)  # Wrap around for circular paths
            if ind == self.current_index:  # Avoid infinite loop in case of very small Lf
                break

        return ind, Lf

    def pure_pursuit_steer_control(self, pind, theta):
        ind = self.search_target_index()[0]

        if pind >= ind:
            ind = pind

        global_tx = self.cx[ind] # This is the target waypoints x position
        global_ty = self.cy[ind] # This is the target waypoints y position

        transform_matrix = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
       
        translation_v = np.array([-self.x, -self.y, 0])

        look_ahead_point_global = np.array([global_tx, global_ty, 0])

        look_ahead_point_robot = transform_matrix @ (look_ahead_point_global + translation_v)

        r = np.linalg.norm(look_ahead_point_robot)

        y = look_ahead_point_robot[1]

        delta = k * 2.0 * y / pow(r, 2) # Angle calculated using CL2 Waterloo p-controllerequation https://github.com/CL2-UWaterloo/f1tenth_ws/blob/main/src/pure_pursuit/src/pure_pursuit.cpp
        # if self.i % 20 == 0:
        self.get_logger().info(f'Delta: {delta}, ind: {ind}')
        return delta, ind

    def drive(self):  # main controller function for the robot

        # Attempt to retrieve the latest transformation
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "map", rclpy.time.Time(), Duration(seconds=1.0)
            )
            # self.get_logger().info(f"Stored transform: translation {transform.transform.translation}")
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get transform between map and odom")
            return  # Skip this loop if transform is not available

        # Use the latest transform to compute the robot’s current position
        self.x = transform.transform.translation.x
        self.y = transform.transform.translation.y
        self.z = transform.transform.translation.z
        self.yaw = tf_transformations.euler_from_quaternion(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        )[2]
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

        if self.initialize == 0:
            self.target_index = self.search_target_index()[0]
        if self.i % 20 == 0:
            self.get_logger().info(f'Target Index: {self.target_index}')
        steering_angle, self.target_index = self.pure_pursuit_steer_control(self.target_index, self.yaw)


        # Calculate steering angle and set velocity
        velocity = self.cv[self.target_index] * throttle_percentage
        
        if self.i % 20 == 0:
            self.get_logger().info(f'Robot x: {self.x}, Point x: {self.cx[self.current_index]}, Robot y: {self.y}, Point y: {self.cy[self.current_index]}')
            self.get_logger().info(f'Current index: {self.current_index}, Target index: {self.target_index}, Steering angle: {steering_angle}')
        self.i += 1   
        
        drive = AckermannDrive(
            steering_angle=steering_angle, speed=velocity
        )

        data = AckermannDriveStamped(header=std_msgs.msg.Header(), drive=drive)
        self.ackermann_publisher.publish(data)

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_controller = PurePursuitController()

    try:
        time.sleep(1)
        rclpy.spin(pure_pursuit_controller)

    except KeyboardInterrupt:
        # handle Ctrl-C
        pass
    # except Exception as e:
    #     pure_pursuit_controller.get_logger().error(
    #         f"Error spinning PurePursuitController class: {e}"
    #     )
    finally:
        # cleanup and shutdown
        pure_pursuit_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
