from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'pure_pursuit_node = f1tenth_controllers.pure_pursuit:main',
            'pure_pursuit_nodev2 = f1tenth_controllers.pure_pursuit_sim:main',
            'p_controller_node = f1tenth_controllers.p_controller:main',
            'mppi_costmap_node = f1tenth_controllers.mppi_goal_kinematic_model_pycuda_costmap:main'
        ],
    },
)
