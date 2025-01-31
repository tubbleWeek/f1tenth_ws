#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
import os

class MapVisualizer:
    def __init__(self, yaml_path, manual_obstacles=None):
        self.manual_obstacles = manual_obstacles or []
        self.load_map_data(yaml_path)
        
    def load_map_data(self, yaml_path):
        # Load YAML file
        with open(yaml_path) as f:
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
        x_min = self.origin[0]
        x_max = self.origin[0] + self.width * self.resolution
        y_min = self.origin[1]
        y_max = self.origin[1] + self.height * self.resolution

        # Plot with correct orientation and scaling
        ax.imshow(self.map_image,
                cmap='gray',
                extent=[x_min, x_max, y_min, y_max],
                origin='lower',
                vmin=0, vmax=100,
                aspect='equal')

    def plot_trajectories(self, ax, trajectories):
        """Plot robot trajectories in world coordinates"""
        for traj in trajectories:
            xs = [p[0] for p in traj]
            ys = [p[1] for p in traj]
            ax.plot(xs, ys, linewidth=2, alpha=0.7)
            ax.scatter(xs[0], ys[0], marker='o', color='green', s=50, label='Start')
            ax.scatter(xs[-1], ys[-1], marker='X', color='red', s=100, label='End')

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
        ax.scatter(-1, -15, color='gold', marker='*', s=200, zorder=5, label='Goal (-1,-15)')

        # Configure axes
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title('Robot Trajectories in Map Coordinates', fontsize=14)
        ax.grid(True, alpha=0.3)
        
        # Set equal aspect ratio and tight bounds
        ax.set_aspect('equal')
        ax.autoscale(tight=True)
        
        # Add legend (only once)
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))  # Deduplicate
        ax.legend(unique.values(), unique.keys(), loc='upper right')

        plt.show()

# Example usage
if __name__ == '__main__':
    # Sample trajectories (replace with your actual data)
    sample_trajectories = [
        [(-0.0, -0.0), (-1.0, -5.5), (-1.0, -10.0)],  # Success case
    ]

    manual_obstacles = [
        # {'x': 0.0, 'y': 0.0, 'width': 0.2, 'height': 1.0},
        {'x': -1.0, 'y': -7.0, 'width': 1.0, 'height': 0.2}
    ]

    visualizer = MapVisualizer(
        yaml_path="/home/nvidia/f1tenth_ws/maps/shepherd_lab_map.yaml",
        manual_obstacles=manual_obstacles
    )
    
    visualizer.visualize(sample_trajectories)