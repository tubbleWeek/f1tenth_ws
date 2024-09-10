import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Quaternion
# import tf_transformations
import scipy
from scipy.spatial.transform import Rotation as R

import sys


import numpy as np
from scipy.ndimage import distance_transform_edt

class LocalMapNode(Node):
    def __init__(self):
        super().__init__('create_localmap')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/local_costmap',
            10)
        
        # Initialize tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define costmap parameters
        self.resolution = 0.05  # meters per cell
        self.width = 4.0  # meters
        self.height = 4.0  # meters
        self.origin_x = -2.0  # meters
        self.origin_y = -2.0  # meters
        self.grid_width = int(self.width / self.resolution)
        self.grid_height = int(self.height / self.resolution)
        self.frame_id = 'base_link'  # Frame of the robot
        self.costmap = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)  # Initialize with -1 for unknown

        # Inflation radius (in meters)
        self.inflation_radius = 0.2  # 20 cm
        self.inflation_cells = int(self.inflation_radius / self.resolution)

        # Car radius
        self.car_radius = 0.3  # Define the car's radius in meters
        
        # Angle range for special marking (in degrees)
        self.angle_min = np.deg2rad(-30)
        self.angle_max = np.deg2rad(30)

    def lidar_callback(self, msg: LaserScan):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'Could not transform laser to base_link: {e}')
            return

        # Reset costmap
        self.costmap.fill(-1)  # Set unknown cells to -1

        # Extract translation and rotation from dynamic transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_matrix = self.get_transform_matrix(translation, rotation)

        # Convert polar coordinates to Cartesian and apply transformation
        for i, distance in enumerate(msg.ranges):
            # Skip readings within the car radius
            if distance < self.car_radius:
                continue

            if msg.range_min < distance < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x_laser = distance * np.cos(angle)
                y_laser = distance * np.sin(angle)

                # Transform to base_link frame
                point_laser = np.array([x_laser, y_laser, 0.0, 1.0])
                point_base_link = np.dot(transform_matrix, point_laser)

                # Convert to costmap indices
                x_base_link = point_base_link[0]
                y_base_link = point_base_link[1]
                map_x = int((x_base_link - self.origin_x) / self.resolution)
                map_y = int((y_base_link - self.origin_y) / self.resolution)

                # Check if the indices are within the costmap bounds
                if 0 <= map_x < self.grid_width and 0 <= map_y < self.grid_height:
                    # Mark with a different value if within the specified angle range
                    if self.angle_min <= angle <= self.angle_max:
                        self.costmap[map_y, map_x] = 50  # Mark with a different value for special range
                    else:
                        self.costmap[map_y, map_x] = 0  # Mark as occupied

        # Perform inflation using Euclidean Distance Transform
        self.inflate_obstacles()

        # Publish the costmap as an OccupancyGrid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = self.frame_id

        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.grid_width
        occupancy_grid.info.height = self.grid_height
        occupancy_grid.info.origin.position.x = self.origin_x
        occupancy_grid.info.origin.position.y = self.origin_y
        occupancy_grid.info.origin.position.z = 0.0

        # Flatten the 2D costmap into a 1D list for the OccupancyGrid
        occupancy_grid.data = [int(cell) for cell in self.costmap.flatten()]
        self.publisher.publish(occupancy_grid)
        self.get_logger().info('Published OccupancyGrid')

    def inflate_obstacles(self):
        '''Version 2'''
        # Compute the distance transform on occupied cells
        distances = distance_transform_edt(self.costmap)

        # Create a mask for cells within the inflation radius
        inflation_mask = distances <= (self.inflation_radius / self.resolution)

        # Calculate inflation costs using linear decay (for cells within the mask)
        inflation_costs = 100 * (1 - (distances / self.inflation_cells))

        # Clip the inflation costs to ensure values are between 0 and 100
        inflation_costs = np.clip(inflation_costs, 0, 100)

        # Apply the inflation costs only to the cells within the inflation mask
        self.costmap = np.maximum(self.costmap, inflation_costs * inflation_mask)

        '''Version 1 - Slow'''
        # # Compute the distance transform
        # distances = distance_transform_edt(self.costmap)
        # # Apply gradient inflation with linear decay
        # for y in range(self.grid_height):
        #     for x in range(self.grid_width):
        #         distance = distances[y, x]
        #         if distance <= self.inflation_radius / self.resolution:
        #             inflation_cost = int(100*(1 - (distance / self.inflation_cells)))
        #             self.costmap[y, x] = max(self.costmap[y, x], inflation_cost)

    def get_transform_matrix(self, translation, rotation):
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 3] = [translation.x, translation.y, translation.z]
        rotation_temp = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        rotation_matrix = rotation_temp.as_matrix()
        transform_matrix[0:3, 0:3] = rotation_matrix
        return transform_matrix

        
def main(args=None):
    rclpy.init(args=args)
    node = LocalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
