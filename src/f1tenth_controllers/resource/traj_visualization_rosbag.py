#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import yaml
import glob
from PIL import Image
import pickle
import os

class MapVisualizer:
    def __init__(self, yaml_path, manual_obstacles=None):
        self.manual_obstacles = manual_obstacles or []
        self.load_map_data(yaml_path)
        
    def load_map_data(self, yaml_path):
        with open(yaml_path) as f: # Load YAML file
            self.map_metadata = yaml.safe_load(f)
        
        # Load and process PGM image
        map_dir = os.path.dirname(yaml_path)
        pgm_path = os.path.join(map_dir, self.map_metadata['image'])
        
        # Flip image vertically to match ROS coordinate convention
        self.map_image = np.flipud(np.array(Image.open(pgm_path)))
        
        # Extract map parameters
        self.resolution = self.map_metadata['resolution']
        self.origin = np.array(self.map_metadata['origin'][:2])  # [x, y] in meters
        self.height, self.width = self.map_image.shape

    def plot_map(self, ax):
        """Plot the occupancy grid map in world coordinates"""
        # Calculate map bounds in meters
        x_min = self.origin[0]  # -16.7 from the YAML
        x_max = self.origin[0] + self.width * self.resolution
        y_min = self.origin[1]  # -20.3 from your YAML
        y_max = self.origin[1] + self.height * self.resolution

        # Define the cropping range (restrict visualization to -10 <= x <= 5)
        crop_x_min = -10
        crop_x_max = 7.5

        # Convert world coordinates to pixel indices
        pixel_x_min = int((crop_x_min - self.origin[0]) / self.resolution)
        pixel_x_max = int((crop_x_max - self.origin[0]) / self.resolution)

        # Ensure valid cropping indices within the image bounds
        pixel_x_min = max(0, pixel_x_min)
        pixel_x_max = min(self.width, pixel_x_max)

        # Crop the image along the X-axis
        cropped_image = self.map_image[:, pixel_x_min:pixel_x_max]

        # Update new world extents after cropping
        new_x_min = crop_x_min
        new_x_max = crop_x_max

        ax.imshow(cropped_image,
                cmap='gray',
                extent=[new_x_min, new_x_max, y_min, y_max],  # New extent
                origin='lower',
                vmin=0, vmax=100,
                aspect='equal')

        # Set new limits to enforce cropping
        ax.set_xlim(new_x_min, new_x_max)

        # # Plot with correct orientation and scaling
        # ax.imshow(self.map_image,
        #         cmap='gray',
        #         extent=[x_min, x_max, y_min, y_max],
        #         origin='lower',
        #         vmin=0, vmax=100,
        #         aspect='equal')
        # ax.set_xlim(crop_x_min, crop_x_max)

    def detect_crash(self, traj):
        # Assume that ending within a certain boundary is a success
        goal_x, goal_y = -1, -15  # Change to actual goal coordinates
        distance_to_goal = np.hypot(traj[-1][0] - goal_x, traj[-1][1] - goal_y)
        return distance_to_goal > 0.5  # Mark as crash if far from goal

    def plot_trajectories(self, ax, trajectories):
        """Plot robot trajectories in world coordinates"""
        for traj in trajectories:
            xs = [p[0] for p in traj]
            ys = [p[1] for p in traj]
            ax.plot(xs, ys, linewidth=2, alpha=0.7)
            ax.scatter(xs[0], ys[0], marker='o', color='green', s=50, label='Start')
            is_crash = self.detect_crash(traj)
            if is_crash:
                ax.scatter(xs[-1], ys[-1], marker='X', color='red', s=100, label='Crash')
            else:
                ax.scatter(xs[-1], ys[-1], marker='^', color='blue', s=50, label='Success')
        ax.scatter([], [], marker='s', color='purple', s=100, label='Obstacle')  

    def plot_obstacles(self, ax):
        """Plot manual obstacles in world coordinates"""
        for obs in self.manual_obstacles:
            ax.add_patch(plt.Rectangle(
                (obs['x'], obs['y']),
                obs['width'],
                obs['height'],
                color='purple',
                alpha=0.5,
                linewidth=2
            ))

    def visualize(self, trajectories):
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Plot map background
        self.plot_map(ax)
        
        # Plot trajectories
        self.plot_trajectories(ax, trajectories)
        
        # Plot manual obstacles
        self.plot_obstacles(ax)
        
        # Add coordinate system elements
        # ax.axhline(0, color='black', linestyle='--', linewidth=0.5)  # X-axis
        # ax.axvline(0, color='black', linestyle='--', linewidth=0.5)  # Y-axis
        ax.scatter(-1, -15, color='gold', marker='*', s=200, zorder=5, label='Goal')

        # Configure axes
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title('Robot Trajectories', fontsize=14)
        ax.grid(True, alpha=0.3)
        
        # Set equal aspect ratio and tight bounds
        ax.set_aspect('equal')
        ax.autoscale(tight=True)
        
        # Add legend (only once)
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))  # Deduplicate
        ax.legend(unique.values(), unique.keys(), loc='upper right')

        plt.show()

if __name__ == '__main__':
    base_path = "/home/nvidia/f1tenth_ws/experiments_data/"
    start_dirs = [
        "cuniform_setting1_starting1_done",
        "cuniform_setting1_starting2_done",
        "cuniform_setting1_starting3_done"
    ]
    all_trajectories = []
    for start_dir in start_dirs:
        traj_files = sorted(glob.glob(os.path.join(base_path, start_dir, "trajectory_*.pkl")))

        for file in traj_files:
            with open(file, 'rb') as f:
                all_trajectories.append(pickle.load(f))  # Append each trajectory set

    manual_obstacles = [
    #     {'x': -1.0, 'y': -7.0, 'width': 1.0, 'height': 0.2}
    ]
    visualizer = MapVisualizer(
        yaml_path="/home/nvidia/f1tenth_ws/maps/shepherd_lab_map.yaml",
        manual_obstacles=manual_obstacles
    )
    visualizer.visualize(all_trajectories)  # Pass all trajectories together