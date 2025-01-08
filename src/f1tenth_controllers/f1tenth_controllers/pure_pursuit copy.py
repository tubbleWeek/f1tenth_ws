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
import time
import numpy as np
import std_msgs
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import math
import tf_transformations


class Waypoint:
    def __init__(self, x, y, velocity, index):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.index = index


class PurePursuitController(Node):
    def __init__(self):
        # member variables
        super().__init__("pure_pursuit_node")
        self.map_path = "/home/nvidia/f1tenth_ws/src/pure_pursuit/racelines/shepherd_lab_raceline_v1.csv"
        self.path = []
        self.x = 0
        self.y = 0
        self.lookahead_dist = 0.55
        self.minlook_ahead = 1
        self.maxlook_ahead = 3
        self.velocity_percentage = 0.2
        self.goal_buble = 0.25
        self.reached_waypoint = False
        self.theta = 0
        self.getPath()

        # ROS subscriptions and publisher
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 20
        )
        self.odom_subscription

        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, "/drive", qos_profile_sensor_data
        )

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.drive)

        # initialization
        self.current_waypoint = Waypoint(self.x, self.y, 4, -1)
        self.lookahead_waypoint = self.getClosestPoint()
        self.steering = self.steering_angle()

    def odom_callback(self, msg):  # update x and y position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

    def getPath(self):  # load all waypoints into list
        waypoints = np.loadtxt(self.map_path, delimiter=",")
        # Load path into waypoint list
        i = 0
        for point in waypoints:
            self.path.append(Waypoint(point[0], point[1], point[2], i))
            i += 1

    def getClosestPoint(
        self,
    ):  # should only be used once in initialization to get first point to go to
        minDistance = (self.path[0].x - self.x) ** 2 + (self.path[0].y - self.y) ** 2
        closestPoint = self.path[0]
        for point in self.path:
            distance = (point.x - self.x) ** 2 + (point.y - self.y) ** 2
            if distance < minDistance:
                minDistance = distance
                closestPoint = point
        return closestPoint

    def get_lookaheadpoint(self):  # function to get the lookahead point
        ret_point = self.path[0]  # Default to the first point if none found
        for point in self.path:
            distance = (point.x - self.x) ** 2 + (point.y - self.y) ** 2
            index_diff = point.index - self.current_waypoint.index
            if (
                distance > self.lookahead_dist
                and self.minlook_ahead <= index_diff <= self.maxlook_ahead
            ):
                ret_point = point
                break
        return ret_point

    def steering_angle(self):
        dx = self.lookahead_waypoint.x - self.x
        dy = self.lookahead_waypoint.y - self.y
        desired_angle = math.atan2(dy, dx)
        angle_diff = desired_angle - self.theta
        max_steering_angle = 0.34
        mapped_angle = (angle_diff / math.pi) * max_steering_angle

        return mapped_angle

    def drive(self):  # main controller function for the robot
        '''
        - check whether you have a current waypoint from lookahead() if no - drive(vel=0,steering_angle=0)
        - calculate the distance between the  robot current position and the current waypoint
        - check whether you reach that waypoint
            - if yes: find use the lookahead function to get the next point
                - calculate the distance again
        - drive to the waypoint
        '''
        
        # if not self.reached_waypoint:
        distance = (self.current_waypoint.x - self.x) ** 2 + (
            self.current_waypoint.y - self.y
        ) ** 2
        if distance < self.goal_buble:
            # self.reached_waypoint = True
            self.current_waypoint = self.lookahead_waypoint
        
        # update lookahead point
        self.lookahead_waypoint = self.get_lookaheadpoint()
        self.get_logger().info(f'lookahead_waypoint: x: {self.lookahead_waypoint.x}, y: {self.lookahead_waypoint.y} ')
        self.steering = self.steering_angle()
        velocity = self.current_waypoint.velocity

        # with 0.2 percentage --> max vel ~ 1m/s
        self.get_logger().info(f"actual velocity: {velocity * self.velocity_percentage}")
        self.get_logger().info(f"velocity: {1.0}")
        self.get_logger().info(f"steering angel: {self.steering}")
        drive = AckermannDrive(
            steering_angle=self.steering, speed=1.0
        )
        # drive = AckermannDrive(
        #     steering_angle=self.steering, speed=velocity * self.velocity_percentage
        # )
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
    except Exception as e:
        pure_pursuit_controller.get_logger().error(
            f"Error spinning PurePursuitController class: {e}"
        )
    finally:
        # cleanup and shutdown
        pure_pursuit_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
