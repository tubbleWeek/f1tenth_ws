from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_controllers'

def get_model_paths():
    model_paths = []
    # Walk through the "models" directory recursively
    for root, _, files in os.walk("models"):
        # For each file, determine its target installation path
        files_list = [
            os.path.join(root, file)
            for file in files
        ]
        if files_list:
            # Target directory: share/<package_name>/<root>
            target_dir = os.path.join("share", package_name, root)
            model_paths.append((target_dir, files_list))
    return model_paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # (os.path.join('share', package_name, 'models'), get_model_paths()),
        *get_model_paths(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='wchastek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_the_gap_node = f1tenth_controllers.follow_the_gap_node:main',
            'path_follower_node = f1tenth_controllers.p_controller:main',
            'mppi_costmap_node = f1tenth_controllers.mppi_dubins_moving_target_ros_node:main',
            'c_uniform_costmap_node = f1tenth_controllers.cuniform_dubins_ros_node:main',
            'comet_costmap_node = f1tenth_controllers.comet_ros2_kinematic:main',
            'stein_costmap_node = f1tenth_controllers.stein_mppi_node:main',
            'pose_collector_node = f1tenth_controllers.amcl_pose_collector_node:main'
        ],
    },
)
