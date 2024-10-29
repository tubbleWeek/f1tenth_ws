#!/usr/bin/env python3
'''
Code copied from Yukang Cao's github repo -> https://github.umn.edu/RSN-Robots/f1tenth_stack/blob/f1tenth_sim_comp/src/controller/controller/gap_follower_node.py
'''
'''
Code modified from Burak Mert Gonultas's code -> https://github.com/gonultasbu/f1tenth_bullet/blob/main/gap_follower.py
'''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float32  # Import the Float32 message type
from sensor_msgs.msg import LaserScan  # Import LaserScan message type
import time
import numpy as np
import random
import std_msgs
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class GapFollowerController(Node):
    def __init__(self):
        super().__init__('gap_follower_contorller')
        """
        Credits: https://github.com/Taeyoung96/f1tenth-my-own-driver/blob/master/pkg/src/pkg/drivers.py
        """
        # self.BUBBLE_RADIUS = 160
        self.BUBBLE_RADIUS = 195
        self.PREPROCESS_CONV_SIZE = 3
        self.BEST_POINT_CONV_SIZE = 80
        self.MAX_LIDAR_DIST = 3000000
        self.STRAIGHTS_STEERING_ANGLE = 0.1 # approaxiamtely 5.7 degree
        # self.max_steering_angle = 0.52 # radians, 30 degree
        # self.min_throttle_scale = 0.3  # Minimum throttle scale when turning

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', qos_profile_sensor_data)


        # Using /drive topic for f1tenth
        # Publisher for /autodrive/f1tenth_1/steering_command topic (Float32)
        #self.steering_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 5)

        # Publisher for /autodrive/f1tenth_1/throttle_command topic (Float32)
        #self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 5)


        self.target_steering_angle = 0.0 # in radians, positive angle -> turn left, negative angle -> turn right
        # 2.0 is meter per seconds longitutional speed desired, this is as fast as you can go in barcelona track for f1tenth without crashing
        self.CORNERS_SPEED = 2.0 / 0.5
        self.target_speed = 2.0 / 0.5  # divide by tire radius to get real speed 
        self.target_throttle = 40 # throttle set to 40 percent 

        # Recovery variables
        self.in_recovery = False
        self.recovery_start_time = None
        self.recovery_duration = 0.5  # seconds

    '''
    The autodrive f1tenth is using lidar with -135 degree to 135 degree, range from 0 to 10m
    '''
    def lidar_callback(self, msg):
        """Callback function to process LiDAR data."""
        ranges = np.array(msg.ranges)  # Extract ranges from LaserScan message and convert to a numpy array

        # Collision detection
        collision_detected = self.check_collision(ranges)

        if collision_detected and not self.in_recovery:
            self.in_recovery = True
            self.recovery_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info('Collision detected. Starting recovery maneuver.')
            
        if self.in_recovery:
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed_time = current_time - self.recovery_start_time
            if elapsed_time < self.recovery_duration:
                # Recovery maneuver: Move backward with negative throttle
                throttle_cmd_value = -0.8  # Negative throttle to move backward
                steering_cmd_value = 0.0 
                # Publish commands
                #steering_cmd = Float32()
                #steering_cmd.data = steering_cmd_value
                steering_cmd = steering_cmd_value
                #self.steering_publisher.publish(steering_cmd)
                #throttle_cmd = Float32()
                #throttle_cmd.data = throttle_cmd_value
                throttle_cmd = throttle_cmd_value
                #self.throttle_publisher.publish(throttle_cmd)
                drive = AckermannDrive(steering_angle = steering_cmd, speed = throttle_cmd)
                data = AckermannDriveStamped(header = std_msgs.msg.Header(), drive = drive)
                self.ackermann_publisher.publish(data)
                self.get_logger().info(f'Recovery mode: Moving backward. Throttle: {throttle_cmd_value:.2f}, Steering: {steering_cmd_value:.2f}')
                return  # Skip the rest of the callback during recovery
            else:
                # Recovery complete
                self.in_recovery = False
                self.get_logger().info('Recovery maneuver complete. Resuming normal operation.')


        ranges_mm = ranges * 1000  # Convert from meters to millimeters
        # Process the LiDAR data to get target speed and steering angle
        self.target_throttle, self.target_steering_angle = self.process_lidar(ranges_mm)
        # if self.target_speed > 2.0/0.5:
        #     self.target_speed = 2.0/0.5
        

        # Calculate the steering command based on the target steering angle
        steering_cmd_value = self.calculate_steering(self.target_steering_angle)

        # Adjust throttle based on steering angle (reduce speed for higher turning rate)
        throttle_cmd_value = self.calculate_throttle(self.target_throttle, self.target_steering_angle)

        # Publish steering command
        #steering_cmd = Float32()
        #steering_cmd.data = steering_cmd_value
        steering_cmd = steering_cmd_value
        #self.steering_publisher.publish(steering_cmd)
        #self.get_logger().info(f'Published steering command: {steering_cmd_value:.2f}')

        # Publish throttle command
        #throttle_cmd = Float32()
        #throttle_cmd.data = throttle_cmd_value
        throttle_cmd = throttle_cmd_value
        #self.throttle_publisher.publish(throttle_cmd)

        # Publishing the commands to the drive topic
        drive = AckermannDrive(steering_angle = steering_cmd, speed = throttle_cmd)
        data = AckermannDriveStamped(header = std_msgs.msg.Header(), drive = drive)
        self.ackermann_publisher.publish(data)
        #self.get_logger().info(f'Published throttle command: {throttle_cmd_value:.2f}')
        self.get_logger().info(f'Target speed: {self.target_speed}, Target steering: {self.target_steering_angle}')
        self.get_logger().info(f'Published drive command: Throttle: {throttle_cmd_value:.2f}, Steering: {steering_cmd_value:.2f}')
        self.get_logger().info(f' ')

    def check_collision(self, ranges):
        """
        Check if there's an obstacle within a threshold distance directly in front of the vehicle.
        """
        threshold_distance = 0.20  # meters
        N = len(ranges)
        angles = np.linspace(-135, 135, N)  # degrees

        # Indices where angle is between -30 and +30 degrees
        front_indices = np.where(np.abs(angles) <= 30)[0]
        front_ranges = ranges[front_indices]
        min_distance = np.min(front_ranges)
        if min_distance < threshold_distance:
            return True
        else:
            return False

    def calculate_throttle(self, target_speed, steering_angle):
        """
        Calculate the throttle command needed to achieve the target speed.
        Adjust throttle based on steering angle.
        """
        # Define throttle values
        max_throttle = 0.40  # Throttle for straight paths
        min_throttle = 0.20  # Throttle for sharp turns
        target_throttle = target_speed/100 # turn target_speed into the target throttle percentage
        # Calculate absolute steering angle
        abs_steering_angle = abs(steering_angle)
        max_steering_angle = 0.5236  # Maximum steering angle in radians

        # Linearly interpolate throttle based on steering angle
        throttle_cmd = max_throttle - (max_throttle - min_throttle) * (abs_steering_angle / max_steering_angle)

        # Ensure throttle is within bounds
        throttle_cmd = max(min(throttle_cmd, max_throttle), min_throttle)

        # Added calculation of the speed in m/s that we want the f1tenth to drive at instead of the throttle percentage.
        throttle_cmd = target_speed * throttle_cmd

        return throttle_cmd

    def calculate_steering(self, target_steering_angle):
        """
        Directly command the target steering angle without simulating steering dynamics.
        """
        # Optionally, you can clip the steering command to the vehicle's max steering angle
        # steering_cmd = max(min(target_steering_angle, self.max_steering_angle), -self.max_steering_angle)
        # return steering_cmd
        target_steering_cmd = max(min(target_steering_angle, 0.5236), -0.5236)
        # edited so no longer returns the percentage and instead the turning command in radians
        steering_cmd = target_steering_cmd #/ 0.5236 # get percentage, based on [-0.5236,0.5236] rad
        steering_cmd *= 0.7 # a scaling factor to limit the turning rate
        return steering_cmd


    def preprocess_lidar(self, ranges):
        """ 
        Preprocess the LiDAR scan array to consider only front-facing data (-90° to +90°).
        """
        N = len(ranges)
        # Indices corresponding to -90° and +90°
        index_start = int(N / 6)      # Corresponds to -90 degrees
        index_end = int(5 * N / 6)    # Corresponds to +90 degrees

        # Slice the ranges to get front-facing data
        front_ranges = ranges[index_start:index_end]

        # Update radians_per_elem based on front_ranges
        self.radians_per_elem = np.pi / len(front_ranges)

        # Process front_ranges
        proc_ranges = np.array(front_ranges)
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ 
		Return the start index & end index of the max gap in free_space_ranges
        free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """
		Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ 
        Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def process_lidar(self, ranges):
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm & publish to /autodrive/f1tenth_1/steering_command and .../throttle_command
        """
        proc_ranges = self.preprocess_lidar(ranges)
        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)
        
        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.target_speed
        # print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        return speed, steering_angle

def main(args=None):
    rclpy.init(args=args)
    gap_follower_controller = GapFollowerController()

    try:
        time.sleep(1)
        rclpy.spin(gap_follower_controller)

    except KeyboardInterrupt:
        # handle Ctrl-C
        pass
    except Exception as e:
        gap_follower_controller.get_logger().error(f"Error spinning GapFollowerController class: {e}")
    finally:
        # cleanup and shutdown
        gap_follower_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
